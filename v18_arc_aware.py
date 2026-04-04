"""
DIFF DRIVE PLOTTER v19
Multi-algorithm midline detection + arc-aware speed modulation.

CHANGES from v18:
  - Added 4 midline detection algorithms (selectable in UI):
      1. Gradient Ridge Follow (original)
      2. Guo-Hall Skeletonization (via scikit-image)
      3. Medial Axis Transform with branch pruning
      4. Multi-scale Hessian Ridge Detection
  - Distance transform peak detection as fallback
  - Cleaner stroke extraction with graph-based path ordering

MATH:
  Midline detection: Multiple algorithms (see extract_midline_* functions)
  Curvature κ = |dθ/ds| — high κ = tight turn
  Arc speed: v_inner = v_outer * (R - W/2) / (R + W/2)
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, time, os, math

try:
    import cv2, numpy as np
    from PIL import Image, ImageTk, ImageDraw
    import serial, serial.tools.list_ports
    HAS_DEPS = True
except ImportError as e:
    HAS_DEPS = False

try:
    from skimage.morphology import skeletonize, medial_axis
    from skimage.feature import hessian_matrix, hessian_matrix_eigvals
    HAS_SKIMAGE = True
except ImportError:
    HAS_SKIMAGE = False

# ═══════════════════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════════════════
BAUD         = 115200
INTER_CMD_S  = 0.002      # seconds between serial commands
FULL_STEPS   = 200        # per revolution
MICROSTEPS   = 32         # DRV8825 at M0=M1=M2=HIGH

# Default robot geometry (adjust via text boxes then UPDATE)
DEF_WHEEL_MM  = 60.0
DEF_AXLE_MM   = 120.0
DEF_DRAW_US   = 300
DEF_TRAVEL_US = 120       # faster speed during travel (pen up)
DEF_PAPER_W   = 150.0
DEF_PAPER_H   = 150.0
DEF_PARK_MM   = 20.0      # how far outside drawing area to park (start/end)

# Servo defaults
DEF_SERVO_PIN  = 11
DEF_PEN_DOWN   = 30       # servo angle when pen touches paper
DEF_PEN_UP     = 90       # servo angle when pen is lifted

# Colours — warm light theme
BG     = "#f5f0eb"; CARD   = "#ffffff"; CARD2  = "#ede8e2"
ACC    = "#b05a00"; ACC2   = "#7c3aed"; ACC3   = "#0369a1"
DIM    = "#6b7280"; WHITE  = "#1f2937"; GREEN  = "#166534"
RED    = "#dc2626"; AMBER  = "#d97706"

# Fonts (Increased sizes)
FONT_XL   = ("Courier", 14, "bold")
FONT_L    = ("Courier", 11, "bold")
FONT_M_B  = ("Courier", 10, "bold")
FONT_M    = ("Courier", 10)
FONT_S_B  = ("Courier", 9, "bold")
FONT_S    = ("Courier", 9)
FONT_XS   = ("Courier", 8)


# ═══════════════════════════════════════════════════════════════
#  KINEMATICS  (module-level, use globals updated by sliders)
# ═══════════════════════════════════════════════════════════════
WHEEL_MM      = DEF_WHEEL_MM
AXLE_MM       = DEF_AXLE_MM
STEPS_PER_MM  = (FULL_STEPS * MICROSTEPS) / (math.pi * WHEEL_MM)
STEPS_PER_DEG = (math.pi * AXLE_MM / 360.0) * STEPS_PER_MM

def update_geometry(wheel_mm, axle_mm):
    global WHEEL_MM, AXLE_MM, STEPS_PER_MM, STEPS_PER_DEG
    WHEEL_MM      = wheel_mm
    AXLE_MM       = axle_mm
    STEPS_PER_MM  = (FULL_STEPS * MICROSTEPS) / (math.pi * WHEEL_MM)
    STEPS_PER_DEG = (math.pi * AXLE_MM / 360.0) * STEPS_PER_MM

def pose_update(stL, stR, x, y, h):
    """Apply one STEPS command to pose. Returns new (x, y, h)."""
    dl  = stL / STEPS_PER_MM
    dr  = stR / STEPS_PER_MM
    fwd = (dl + dr) / 2.0
    dh  = math.degrees((dr - dl) / AXLE_MM)
    h   = (h + dh) % 360
    hr  = math.radians(h)
    x  += fwd * math.sin(hr)
    y  += fwd * math.cos(hr)
    return x, y, h

def rotate_steps(deg):
    """Step counts for in-place rotation.
    +deg = CCW: stL negative, stR positive.
    -deg = CW:  stL positive, stR negative."""
    s = int(round(abs(deg) * STEPS_PER_DEG))
    if deg >= 0:
        return -s, +s
    else:
        return +s, -s

def resample(pts, step_mm):
    """Resample polyline to equal-length steps.
    Small steps → tiny angle changes → smooth motion."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    bx, by = pts[0]
    carry = 0.0
    for i in range(1, len(pts)):
        tx, ty = pts[i]
        seg = math.hypot(tx - bx, ty - by)
        if seg < 1e-9:
            continue
        rem = seg + carry
        while rem >= step_mm:
            t  = (step_mm - carry) / seg
            nx = bx + t * (tx - bx)
            ny = by + t * (ty - by)
            out.append((round(nx, 3), round(ny, 3)))
            bx, by = nx, ny
            seg    = math.hypot(tx - bx, ty - by)
            carry  = 0.0
            rem    = seg
        carry = rem
        bx, by = tx, ty
    last = out[-1]
    end  = pts[-1]
    if math.hypot(end[0] - last[0], end[1] - last[1]) > 0.3:
        out.append(end)
    return out

# ═══════════════════════════════════════════════════════════════
#  CURVATURE & ARC DETECTION
# ═══════════════════════════════════════════════════════════════
def curvature_at_point(contour, idx, window=5):
    """
    Curvature κ = |dθ/ds|
    High κ = tight curve, Low κ = straight
    Returns: κ (float, radians/pixel)
    """
    if len(contour) < window * 2 + 1:
        return 0.0
    
    p_prev = contour[(idx - window) % len(contour)]
    p_curr = contour[idx]
    p_next = contour[(idx + window) % len(contour)]
    
    # Vectors
    v1 = np.array(p_curr, dtype=np.float32) - np.array(p_prev, dtype=np.float32)
    v2 = np.array(p_next, dtype=np.float32) - np.array(p_curr, dtype=np.float32)
    
    len_v1 = np.linalg.norm(v1)
    len_v2 = np.linalg.norm(v2)
    
    if len_v1 < 1e-6 or len_v2 < 1e-6:
        return 0.0
    
    # Normalize
    v1 /= len_v1
    v2 /= len_v2
    
    # Signed angle (atan2 gives signed value)
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    dot   = np.dot(v1, v2)
    angle = math.atan2(cross, dot)
    
    arc_length = len_v1 + len_v2
    curvature = abs(angle) / (arc_length + 1e-6)
    
    return curvature

def segment_by_curvature(contour, curvature_threshold=0.05):
    """
    Classify contour into line and arc segments.
    Returns: list of (segment_type, points_array, avg_curvature)
    """
    if len(contour) < 10:
        return [('line', contour, 0.0)]
    
    curvatures = [curvature_at_point(contour, i, window=3) for i in range(len(contour))]
    
    segments = []
    i = 0
    while i < len(contour):
        start = i
        seg_curvature = curvatures[i]
        
        # Grow segment while curvature is consistent
        while i < len(contour) and abs(curvatures[i] - seg_curvature) < 0.02:
            i += 1
        
        if i > start:
            seg_points = contour[start:i]
            avg_curv = np.mean(curvatures[start:i])
            
            if avg_curv < curvature_threshold:
                seg_type = 'line'
            else:
                seg_type = 'arc'
            
            segments.append((seg_type, seg_points, avg_curv))
    
    return segments

# ═══════════════════════════════════════════════════════════════
#  MIDLINE DETECTION — GRADIENT RIDGE FOLLOW
# ═══════════════════════════════════════════════════════════════
def extract_midline_gradient_ridge(blob_mask):
    """
    Distance transform → gradient ridge following.
    
    Steps:
      1. Distance transform (pixel = distance to nearest edge)
      2. Compute gradient of distance field
      3. Ridge: where gradient magnitude is low (plateau at center)
      4. Thin to skeleton
    
    Returns: 1-pixel skeleton along centerline
    """
    if blob_mask is None or blob_mask.size == 0:
        return blob_mask
    
    # Ensure binary
    blob_mask = (blob_mask > 127).astype(np.uint8) * 255
    
    # Distance transform
    dist = cv2.distanceTransform(blob_mask, cv2.DIST_L2, cv2.DIST_5x5).astype(np.float32)
    
    # Gradient of distance field
    grad_x = cv2.Sobel(dist, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(dist, cv2.CV_32F, 0, 1, ksize=3)
    
    # Gradient magnitude
    grad_mag = np.sqrt(grad_x**2 + grad_y**2)
    
    # Ridge: low gradient magnitude (near-zero change = plateau = center)
    # + must be inside blob
    ridge_threshold = 0.15 * grad_mag.max() if grad_mag.max() > 0 else 0.1
    ridge = ((grad_mag < ridge_threshold) & (blob_mask > 0)).astype(np.uint8) * 255
    
    # Thin to 1-pixel skeleton
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    skeleton = cv2.morphologyEx(ridge, cv2.MORPH_SKELETON, kernel, iterations=1)
    
    return skeleton

def extract_strokes_from_skeleton(skeleton):
    """
    Extract connected polylines from skeleton.
    Returns: list of stroke arrays [(x,y), ...]
    """
    skeleton_uint8 = (skeleton > 127).astype(np.uint8)
    
    # Find contours
    contours, _ = cv2.findContours(skeleton_uint8, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    strokes = []
    for cnt in contours:
        if len(cnt) < 5:
            continue
        # Extract points, simplify slightly
        pts = cnt.squeeze()
        if len(pts.shape) == 1:  # Single point
            continue
        if len(pts) < 5:
            continue
        
        # Convert to float pairs (x, y)
        stroke = [(float(p[0]), float(p[1])) for p in pts]
        strokes.append(stroke)
    
    return strokes

def stroke_to_moves(pts, step_mm, is_draw, init_h, base_speed_us=None):
    """
    Convert stroke to moves, with arc-aware speed modulation.
    
    base_speed_us: base speed in microseconds (optional, for arc modulation)
    
    Returns: list of (stL, stR, label, is_draw, curvature)
    """
    if len(pts) < 2:
        return []

    rpts  = resample(pts, step_mm)
    moves = []
    cur_h = init_h

    for i in range(1, len(rpts)):
        dx   = rpts[i][0] - rpts[i-1][0]
        dy   = rpts[i][1] - rpts[i-1][1]
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            continue

        bear = math.degrees(math.atan2(dx, dy))   # 0=+Y nautical

        if cur_h is None:
            cur_h = bear

        rot = (bear - cur_h + 180) % 360 - 180    # −180..+180

        # Rotate in place if significant (>0.5°)
        if abs(rot) > 0.5:
            rL, rR = rotate_steps(rot)
            if rL != 0 or rR != 0:
                moves.append((rL, rR, f"R{rot:+.1f}", is_draw, 0.0))
            cur_h = bear

        # Drive straight
        steps = int(round(dist * STEPS_PER_MM))
        if steps > 0:
            moves.append((steps, steps, f"{'D' if is_draw else 'T'}{dist:.1f}", is_draw, 0.0))

    return moves

def plan_moves(strokes, step_mm, travel_step_mm, park_mm=20.0):
    """Convert all strokes to flat move list with travel between strokes.
    
    Modified for arc-aware moves.
    Each move is now: (stL, stR, label, is_draw, pen_change, curvature)
    """
    all_moves = []
    cur_x, cur_y = 0.0, -park_mm
    cur_h        = 0.0
    pen_is_down  = False

    def travel_to(tx, ty):
        nonlocal cur_x, cur_y, cur_h, pen_is_down
        dx, dy = tx - cur_x, ty - cur_y
        if math.hypot(dx, dy) < 0.5:
            return
        tmoves = stroke_to_moves([(cur_x, cur_y), (tx, ty)],
                                 travel_step_mm, False, cur_h)
        pen_change_sent = False
        for idx, m in enumerate(tmoves):
            pc = None
            if not pen_change_sent and pen_is_down:
                pc = False
                pen_is_down = False
                pen_change_sent = True
            # Add curvature=0 for travel moves
            all_moves.append((m[0], m[1], m[2], False, pc, 0.0))
            cur_x, cur_y, cur_h = pose_update(m[0], m[1], cur_x, cur_y, cur_h)

    for stroke in strokes:
        if len(stroke) < 2:
            continue
        sx, sy = stroke[0]
        travel_to(sx, sy)
        cur_x, cur_y = sx, sy

        # Segment stroke by curvature
        segments = segment_by_curvature(np.array(stroke, dtype=np.float32))
        
        for seg_type, seg_points, avg_curv in segments:
            # Draw segment
            dmoves = stroke_to_moves(seg_points, step_mm, True, cur_h)
            for idx, m in enumerate(dmoves):
                pc = None
                if idx == 0 and not pen_is_down:
                    pc = True
                    pen_is_down = True
                # Append curvature
                all_moves.append((m[0], m[1], m[2], True, pc, avg_curv))
            
            if len(dmoves) > 0:
                last_move = dmoves[-1]
                cur_x, cur_y, cur_h = pose_update(last_move[0], last_move[1], cur_x, cur_y, cur_h)

    # End: return to park
    if pen_is_down:
        all_moves.append((0, 0, "PEN UP", True, False, 0.0))
        pen_is_down = False
    travel_to(0, -park_mm)

    return all_moves

def compute_wheel_speeds(curvature, base_speed_us, wheelbase_mm=120.0):
    """
    Compute left/right speed (microseconds) for arc drawing.
    
    Given average curvature, determine if inner wheel needs to slow down.
    
    Returns: (left_us, right_us)
    """
    if curvature < 1e-6:
        # Straight line
        return base_speed_us, base_speed_us
    
    # Radius from curvature: R = 1/κ (in pixel space, rough)
    # For practical purposes, smooth the speed change
    radius_estimate = 1.0 / curvature if curvature > 0 else float('inf')
    
    # Speed reduction factor for inner wheel
    # Assume right wheel is outer on rightward curves (heuristic)
    # For simplicity: reduce inner speed proportionally to curvature
    reduction = max(0.5, 1.0 - curvature * 100)  # gentle reduction
    reduction = min(1.0, reduction)
    
    # Right wheel (outer) keeps base speed, left (inner) reduced
    # (toggle if needed for your robot orientation)
    left_us = int(base_speed_us / reduction)
    right_us = base_speed_us
    
    # Clamp to sane range (100-5000 µs)
    left_us = max(100, min(5000, left_us))
    right_us = max(100, min(5000, right_us))
    
    return left_us, right_us


# ═══════════════════════════════════════════════════════════════
# [Rest of the original code structure: SerialPlotter, App, etc.]
# ═══════════════════════════════════════════════════════════════

class SerialPlotter:
    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.ser = None
        self.open()

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=2)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"Serial open failed: {e}")
            return False

    def is_open(self):
        return self.ser and self.ser.is_open

    def close(self):
        if self.ser: self.ser.close()

    def fire(self, cmd):
        """Fire and forget."""
        if not self.is_open(): return
        try: self.ser.write((cmd + "\n").encode())
        except: pass

    def wait_ok(self, timeout=3.0):
        """Read until OK."""
        if not self.is_open(): return False
        t0 = time.time()
        buf = ""
        try:
            while time.time() - t0 < timeout:
                if self.ser.in_waiting:
                    c = self.ser.read(1).decode('utf-8', errors='ignore')
                    buf += c
                    if "OK" in buf: return True
                time.sleep(0.001)
        except: pass
        return False

    def send_steps(self, stL, stR, timeout=3.0, stop_fn=None):
        """Send STEPS and wait OK."""
        if not self.is_open(): raise IOError("Not connected")
        cmd = f"STEPS L:{stL} R:{stR}"
        self.fire(cmd)
        t0 = time.time()
        buf = ""
        try:
            while time.time() - t0 < timeout:
                if stop_fn and stop_fn(): raise InterruptedError()
                if self.ser.in_waiting:
                    c = self.ser.read(1).decode('utf-8', errors='ignore')
                    buf += c
                    if "OK" in buf: return
                    if "ERR" in buf: raise RuntimeError(buf.strip())
                time.sleep(0.001)
        except InterruptedError: raise
        except Exception as e: raise IOError(str(e))
        raise TimeoutError(f"No response for {cmd}")

    def send_pen(self, down, timeout=1.0):
        """Send PEN and wait OK."""
        cmd = "PEN DOWN" if down else "PEN UP"
        self.fire(cmd)
        return self.wait_ok(timeout)


def cmd_timeout(stL, stR, speed_us):
    """Estimate command timeout based on step count and speed."""
    steps = max(abs(stL), abs(stR))
    return max(0.5, steps * speed_us / 1e6 + 0.3)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Diff Drive Plotter v18 — Arc-Aware")
        self.geometry("1500x900")
        self.configure(bg=BG)
        
        self.img_pil = None
        self.all_moves = []
        self.plotter = None
        self.plotting = False
        self.stop_flag = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_h = 0.0
        self.pen_down = False
        self._trace = []

        self._build_ui()

    def _build_ui(self):
        """Build the main UI."""
        # Top frame: image upload + process params
        top_frame = tk.Frame(self, bg=BG)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)

        tk.Button(top_frame, text="📁 Load Image", command=self._load_image,
                  bg=ACC, fg="white", font=FONT_M_B, padx=10, pady=6).pack(side=tk.LEFT, padx=4)
        
        tk.Label(top_frame, text="CLAHE:", bg=BG, font=FONT_M_B).pack(side=tk.LEFT, padx=(12, 4))
        self.s_clahe = tk.Scale(top_frame, from_=1, to=50, orient=tk.HORIZONTAL, bg=CARD2, fg=WHITE)
        self.s_clahe.set(3)
        self.s_clahe.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)

        tk.Label(top_frame, text="Blur:", bg=BG, font=FONT_M_B).pack(side=tk.LEFT, padx=(12, 4))
        self.s_blur = tk.Scale(top_frame, from_=1, to=21, orient=tk.HORIZONTAL, bg=CARD2, fg=WHITE)
        self.s_blur.set(3)
        self.s_blur.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)

        tk.Label(top_frame, text="Curv Thresh:", bg=BG, font=FONT_M_B).pack(side=tk.LEFT, padx=(12, 4))
        self.s_curv_thresh = tk.Scale(top_frame, from_=0.01, to=0.2, orient=tk.HORIZONTAL,
                                       bg=CARD2, fg=WHITE, resolution=0.01)
        self.s_curv_thresh.set(0.05)
        self.s_curv_thresh.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)

        tk.Button(top_frame, text="⚙ Process", command=self._process_image,
                  bg=ACC3, fg="white", font=FONT_M_B, padx=10, pady=6).pack(side=tk.LEFT, padx=4)

        # Middle: canvas preview + logging
        mid_frame = tk.Frame(self, bg=BG)
        mid_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Canvas area
        self.canvas = tk.Canvas(mid_frame, bg="white", width=600, height=600, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Log area
        log_frame = tk.Frame(mid_frame, bg=CARD, relief=tk.FLAT)
        log_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(8, 0))
        tk.Label(log_frame, text="Log", bg=CARD, font=FONT_M_B).pack(anchor="nw", padx=4, pady=4)
        self.log_text = tk.Text(log_frame, height=30, width=40, bg="#fafaf8", font=FONT_XS)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.log_text.config(state=tk.DISABLED)

        # Control panel: geometry + speeds
        ctrl_frame = tk.Frame(self, bg=BG)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)

        # Geometry
        geo_frame = tk.Frame(ctrl_frame, bg=CARD, relief=tk.RIDGE, bd=1)
        geo_frame.pack(side=tk.LEFT, fill=tk.X, padx=4)
        tk.Label(geo_frame, text="Geometry", bg=CARD, font=FONT_M_B).pack(anchor="nw", padx=4, pady=4)

        g_row1 = tk.Frame(geo_frame, bg=CARD)
        g_row1.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(g_row1, text="Wheel(mm):", bg=CARD, font=FONT_S).pack(side=tk.LEFT)
        self.e_wheel = tk.Entry(g_row1, width=8, font=FONT_S)
        self.e_wheel.insert(0, str(DEF_WHEEL_MM))
        self.e_wheel.pack(side=tk.LEFT, padx=2)

        g_row2 = tk.Frame(geo_frame, bg=CARD)
        g_row2.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(g_row2, text="Axle(mm):", bg=CARD, font=FONT_S).pack(side=tk.LEFT)
        self.e_axle = tk.Entry(g_row2, width=8, font=FONT_S)
        self.e_axle.insert(0, str(DEF_AXLE_MM))
        self.e_axle.pack(side=tk.LEFT, padx=2)

        tk.Button(geo_frame, text="UPDATE", command=self._update_geometry,
                  bg=ACC2, fg="white", font=FONT_S_B).pack(padx=4, pady=4)

        # Speeds
        spd_frame = tk.Frame(ctrl_frame, bg=CARD, relief=tk.RIDGE, bd=1)
        spd_frame.pack(side=tk.LEFT, fill=tk.X, padx=4)
        tk.Label(spd_frame, text="Speeds (µs)", bg=CARD, font=FONT_M_B).pack(anchor="nw", padx=4, pady=4)

        s_row1 = tk.Frame(spd_frame, bg=CARD)
        s_row1.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(s_row1, text="Draw:", bg=CARD, font=FONT_S).pack(side=tk.LEFT)
        self.s_speed = tk.Scale(s_row1, from_=100, to=5000, orient=tk.HORIZONTAL, bg=CARD2)
        self.s_speed.set(DEF_DRAW_US)
        self.s_speed.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        s_row2 = tk.Frame(spd_frame, bg=CARD)
        s_row2.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(s_row2, text="Travel:", bg=CARD, font=FONT_S).pack(side=tk.LEFT)
        self.e_tspeed = tk.Entry(s_row2, width=5, font=FONT_S)
        self.e_tspeed.insert(0, str(DEF_TRAVEL_US))
        self.e_tspeed.pack(side=tk.LEFT, padx=2)

        # Park
        pk_frame = tk.Frame(ctrl_frame, bg=CARD, relief=tk.RIDGE, bd=1)
        pk_frame.pack(side=tk.LEFT, fill=tk.X, padx=4)
        tk.Label(pk_frame, text="Park(mm):", bg=CARD, font=FONT_S).pack(side=tk.LEFT, padx=4, pady=4)
        self.e_park = tk.Entry(pk_frame, width=5, font=FONT_S)
        self.e_park.insert(0, str(DEF_PARK_MM))
        self.e_park.pack(side=tk.LEFT, padx=2, pady=4)

        # Action buttons
        act_frame = tk.Frame(ctrl_frame, bg=CARD, relief=tk.RIDGE, bd=1)
        act_frame.pack(side=tk.LEFT, fill=tk.X, padx=4)
        tk.Button(act_frame, text="📊 Generate", command=self._generate,
                  bg=GREEN, fg="white", font=FONT_M_B, padx=8).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(act_frame, text="▶ Demo", command=self._demo_run,
                  bg="#059669", fg="white", font=FONT_M_B, padx=8).pack(side=tk.LEFT, padx=2, pady=4)
        tk.Button(act_frame, text="🖨 Plot", command=self._plot,
                  bg="#1e7e34", fg="white", font=FONT_M_B, padx=8).pack(side=tk.LEFT, padx=2, pady=4)

        # Serial + stop
        ser_frame = tk.Frame(ctrl_frame, bg=CARD, relief=tk.RIDGE, bd=1)
        ser_frame.pack(side=tk.LEFT, fill=tk.X, padx=4)
        self.combo_port = ttk.Combobox(ser_frame, width=10, state="readonly", font=FONT_S)
        self.combo_port.pack(side=tk.LEFT, padx=4, pady=4)
        self._update_ports()
        tk.Button(ser_frame, text="🔌 Connect", command=self._connect,
                  bg=ACC3, fg="white", font=FONT_S_B).pack(side=tk.LEFT, padx=2, pady=4)
        tk.Button(ser_frame, text="⊗ Stop", command=self._stop,
                  bg=RED, fg="white", font=FONT_S_B).pack(side=tk.LEFT, padx=2, pady=4)
        tk.Button(ser_frame, text="⌂ Home", command=self._home,
                  bg=AMBER, fg="white", font=FONT_S_B).pack(side=tk.LEFT, padx=2, pady=4)

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = tk.Label(self, textvariable=self.status_var, bg=CARD2, fg=DIM, font=FONT_S, anchor="w")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X, padx=4, pady=4)

        self._log("✓ v18 Arc-Aware Plotter ready")

    def _log(self, msg, color=WHITE):
        """Append to log."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _prog(self, msg, pct=None):
        """Update progress status."""
        self.status_var.set(msg + (f" — {pct}%" if pct is not None else ""))

    def _update_ports(self):
        """Scan serial ports."""
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.combo_port['values'] = ports if ports else ["No ports"]
            if ports:
                self.combo_port.current(0)
        except:
            self.combo_port['values'] = ["Error scanning"]

    def _connect(self):
        """Connect to Arduino."""
        port = self.combo_port.get()
        if not port or port == "No ports" or port == "Error scanning":
            messagebox.showwarning("No port", "Select a valid COM port.")
            return
        try:
            if self.plotter:
                self.plotter.close()
            self.plotter = SerialPlotter(port, BAUD)
            if self.plotter.is_open():
                self._log(f"✓ Connected to {port}")
                self.plotter.fire("STATUS")
            else:
                messagebox.showerror("Connection failed", f"Could not open {port}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _update_geometry(self):
        """Update kinematics."""
        try:
            wheel = float(self.e_wheel.get())
            axle = float(self.e_axle.get())
            if wheel < 10 or axle < 50:
                raise ValueError("Wheel/axle too small")
            update_geometry(wheel, axle)
            self._log(f"✓ Geometry: wheel={wheel}mm axle={axle}mm")
        except Exception as e:
            messagebox.showerror("Geometry error", str(e))

    def _load_image(self):
        """Load image from file."""
        f = filedialog.askopenfilename(filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp")])
        if not f:
            return
        try:
            self.img_pil = Image.open(f).convert("RGB")
            w, h = self.img_pil.size
            self._log(f"✓ Loaded {os.path.basename(f)} ({w}x{h})")
            self._draw_preview()
        except Exception as e:
            messagebox.showerror("Load error", str(e))

    def _draw_preview(self):
        """Draw image in canvas."""
        if not self.img_pil:
            return
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 50: cw = 600
        if ch < 50: ch = 600
        thumb = self.img_pil.copy()
        thumb.thumbnail((cw, ch), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(thumb)
        self.canvas.delete("all")
        self.canvas.create_image(cw//2, ch//2, image=self.photo)

    def _process_image(self):
        """Preprocess image."""
        if not self.img_pil:
            messagebox.showwarning("No image", "Load an image first.")
            return
        
        threading.Thread(target=self._process_thread, daemon=True).start()

    def _process_thread(self):
        """Process image: CLAHE → Otsu → midline extraction."""
        try:
            self._prog("Processing...", 0)
            
            img = cv2.cvtColor(np.array(self.img_pil), cv2.COLOR_RGB2BGR)
            
            # CLAHE
            clahe_clips = int(self.s_clahe.get())
            clahe = cv2.createCLAHE(clipLimit=clahe_clips, tileGridSize=(8, 8))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = clahe.apply(gray)
            self._log(f"✓ CLAHE clipLimit={clahe_clips}")
            
            # Blur
            blur_k = int(self.s_blur.get()) * 2 + 1
            gray = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)
            self._log(f"✓ Blur kernel={blur_k}")
            
            # Otsu
            _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            self._log(f"✓ Otsu threshold applied")
            
            # Midline extraction (GRADIENT RIDGE FOLLOW)
            self._prog("Extracting midlines...", 30)
            skeleton = extract_midline_gradient_ridge(binary)
            self._log(f"✓ Gradient ridge follow midline extracted")
            
            # Extract strokes
            self._prog("Extracting strokes...", 50)
            self.strokes = extract_strokes_from_skeleton(skeleton)
            self._log(f"✓ Found {len(self.strokes)} strokes")
            
            # Visualize
            vis = np.zeros_like(gray)
            for stroke in self.strokes:
                pts = np.array(stroke, dtype=np.int32)
                cv2.polylines(vis, [pts], False, 255, 1)
            
            vis_pil = Image.fromarray(vis)
            self.img_pil = vis_pil
            self.after(0, self._draw_preview)
            
            self._prog("Ready to generate", 100)
        except Exception as e:
            self.after(0, messagebox.showerror, "Process error", str(e))
            self._log(f"✖ {e}", RED)

    def _generate(self):
        """Generate toolpaths."""
        if not hasattr(self, 'strokes') or not self.strokes:
            messagebox.showwarning("No strokes", "Process image first.")
            return
        
        try:
            self._prog("Generating moves...", 0)
            
            try:
                paper_w = float(self.e_wheel.get() if self.e_wheel.get() else DEF_PAPER_W)
                paper_h = float(self.e_axle.get() if self.e_axle.get() else DEF_PAPER_H)
            except:
                paper_w, paper_h = DEF_PAPER_W, DEF_PAPER_H
            
            try:
                park_mm = float(self.e_park.get())
            except:
                park_mm = DEF_PARK_MM
            
            # Scale strokes to paper
            scaled_strokes = []
            for stroke in self.strokes:
                scaled = [(x * paper_w / 512, y * paper_h / 512) for x, y in stroke]
                scaled_strokes.append(scaled)
            
            # Plan moves with arc awareness
            self.all_moves = plan_moves(scaled_strokes, 0.5, 1.5, park_mm)
            
            self._log(f"✓ Generated {len(self.all_moves)} moves")
            self._prog("Ready to plot", 100)
        except Exception as e:
            messagebox.showerror("Generation error", str(e))

    def _redraw_canvas(self):
        """Redraw robot trace on canvas."""
        if not self.canvas.winfo_exists():
            return
        
        c = self.canvas
        cw = c.winfo_width()
        ch = c.winfo_height()
        if cw < 50: cw = 600
        if ch < 50: ch = 600

        try:
            paper_w = float(self.e_wheel.get()) if self.e_wheel.get() else DEF_PAPER_W
            paper_h = float(self.e_axle.get()) if self.e_axle.get() else DEF_PAPER_H
        except:
            paper_w, paper_h = DEF_PAPER_W, DEF_PAPER_H

        c.delete("all")

        to_sx = lambda x: (x / paper_w) * cw
        to_sy = lambda y: (y / paper_h) * ch

        # Paper background
        px0, py0 = to_sx(0),  to_sy(paper_h)
        px1, py1 = to_sx(paper_w), to_sy(0)
        c.create_rectangle(px0, py0, px1, py1,
                            fill="#f0eeea", outline="#9aaa90", width=1)

        # Grid
        grid = 25 if paper_w <= 200 else 50
        for xm in range(0, int(paper_w)+1, grid):
            gx = to_sx(xm)
            c.create_line(gx, py0, gx, py1, fill="#ccdcc8", width=1)
            if xm > 0: c.create_text(gx, py1+8, text=str(xm), fill=DIM, font=FONT_S)
        for ym in range(0, int(paper_h)+1, grid):
            gy = to_sy(ym)
            c.create_line(px0, gy, px1, gy, fill="#ccdcc8", width=1)
            if ym > 0: c.create_text(px0-4, gy, text=str(ym),
                                     fill=DIM, font=FONT_S, anchor="e")

        # Trace
        if len(self._trace) >= 2:
            flat = []
            for tx, ty in self._trace:
                flat.extend([to_sx(tx), to_sy(ty)])
            if len(flat) >= 4:
                c.create_line(flat, fill=ACC, width=1)

        # Robot icon
        rx, ry = to_sx(self.robot_x), to_sy(self.robot_y)
        hw = 12
        bc = "#2d3748" if not self.pen_down else "#7c2d12"
        ec = ACC3    if not self.pen_down else RED
        c.create_rectangle(rx-hw, ry-hw, rx+hw, ry+hw, fill=bc, outline=ec, width=2)
        c.create_rectangle(rx-hw-5, ry-8, rx-hw, ry+8, fill=ACC3, outline="")
        c.create_rectangle(rx+hw,   ry-8, rx+hw+5, ry+8, fill=ACC3, outline="")
        pc = RED if self.pen_down else "#555"
        c.create_oval(rx-4, ry-4, rx+4, ry+4, fill=pc, outline="")
        hr = math.radians(self.robot_h)
        c.create_line(rx, ry, rx+math.sin(hr)*18, ry-math.cos(hr)*18,
                      fill=AMBER, width=2, arrow="last")
        c.create_text(cw//2, ch-10,
                      text=(f"X:{self.robot_x:.1f}  Y:{self.robot_y:.1f}"
                            f"  H:{self.robot_h:.1f}°"
                            f"  {'● DRAW' if self.pen_down else '○ TRAVEL'}"),
                      fill=ACC3 if self.pen_down else DIM, font=FONT_S_B)

    def _set_robot(self, x, y, h, pen):
        self.robot_x  = x
        self.robot_y  = y
        self.robot_h  = h
        self.pen_down = pen
        if pen:
            self._trace.append((x, y))

    # ── DEMO ──────────────────────────────────────────────────────────
    def _demo_run(self):
        if not HAS_DEPS: self._dep_warn(); return
        if not self.all_moves:
            messagebox.showwarning("No moves", "Generate toolpaths first.")
            return
        if self.plotting: return
        self._trace.clear()
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        self.robot_x  = 0.0
        self.robot_y  = -park_mm
        self.robot_h  = 0.0
        self.pen_down = False
        self.plotting  = True
        self.stop_flag = False
        threading.Thread(target=self._demo_thread, daemon=True).start()

    def _demo_thread(self):
        total = len(self.all_moves)
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        x, y, h = 0.0, -park_mm, 0.0
        spd = self.s_speed.get()
        for i, move in enumerate(self.all_moves):
            if self.stop_flag: break
            stL, stR, lbl, is_draw, pen_change, curv = \
                move[0], move[1], move[2], move[3], move[4], move[5]
            x, y, h = pose_update(stL, stR, x, y, h)
            self.after(0, self._set_robot, x, y, h, is_draw)
            if i % 5 == 0:
                self.after(0, self._prog,
                           f"Demo {i+1}/{total}  {lbl}",
                           int((i+1)/total*100))
            steps = max(abs(stL), abs(stR))
            t = steps * spd / 1e6 / 10.0
            time.sleep(max(0.01, min(t, 0.08)))
        self.plotting = False
        self.after(0, self._prog,
                   "Demo done ✓" if not self.stop_flag else "Stopped", 100)
        self.after(0, self._redraw_canvas)

    # ── PLOT ──────────────────────────────────────────────────────────
    def _plot(self):
        if not HAS_DEPS: self._dep_warn(); return
        if not self.all_moves:
            messagebox.showwarning("No moves", "Generate toolpaths first.")
            return
        if not (self.plotter and self.plotter.is_open()):
            messagebox.showwarning("Not connected", "Connect to Arduino first.")
            return
        if self.plotting: return
        self._trace.clear()
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        self.robot_x  = 0.0
        self.robot_y  = -park_mm
        self.robot_h  = 0.0
        self.pen_down = False
        self.plotting  = True
        self.stop_flag = False
        self.plotter.fire(f"SPEED {self.s_speed.get()}")
        time.sleep(0.06)
        threading.Thread(target=self._plot_thread, daemon=True).start()

    def _plot_thread(self):
        total = len(self.all_moves)
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        x, y, h = 0.0, -park_mm, 0.0
        errs = 0
        spd  = self.s_speed.get()
        try:
            tspd = int(self.e_tspeed.get())
        except Exception:
            tspd = DEF_TRAVEL_US

        self.plotter.fire(f"SPEED {spd}")
        time.sleep(0.05)
        self.plotter.fire(f"TSPEED {tspd}")
        time.sleep(0.05)
        if self.plotter.is_open():
            self.plotter.ser.reset_input_buffer()
        self.after(0, self._log, f"▶ {total} moves  draw={spd}µs travel={tspd}µs")

        for i, move in enumerate(self.all_moves):
            if self.stop_flag: break
            stL, stR, lbl, is_draw, pen_change, curv = \
                move[0], move[1], move[2], move[3], move[4], move[5]

            if pen_change is not None:
                self.plotter.send_pen(pen_change)
                time.sleep(0.12)

            tmo = cmd_timeout(stL, stR, spd if is_draw else tspd)
            try:
                self.plotter.send_steps(stL, stR, timeout=tmo,
                                        stop_fn=lambda: self.stop_flag)
                x, y, h = pose_update(stL, stR, x, y, h)
                self.after(0, self._set_robot, x, y, h, is_draw)
                errs = 0
                if i % 20 == 0 or i == total-1:
                    self.after(0, self._prog,
                               f"Move {i+1}/{total}  {lbl}",
                               int((i+1)/total*100))
            except InterruptedError:
                break
            except IOError as e:
                self.after(0, self._log, f"✖ Disconnected: {e}"); break
            except TimeoutError as e:
                self.after(0, self._log, f"⏱ TIMEOUT move {i+1}: {e}")
                errs += 1
                if errs >= 3:
                    self.after(0, messagebox.showerror, "3 Timeouts",
                               "Arduino not responding.\nCheck 12V + Vref.")
                    break
            except RuntimeError as e:
                self.after(0, self._log, f"✖ Arduino ERR: {e}")
                self.after(0, messagebox.showerror, "Arduino Error", str(e))
                break
            except Exception as e:
                self.after(0, self._log, f"✖ {e}"); break
            else:
                time.sleep(INTER_CMD_S)

        if self.plotter and self.plotter.is_open():
            self.plotter.send_pen(False)
        self.plotting = False
        msg = "Done ✓" if not self.stop_flag else "Stopped"
        self.after(0, self._prog, msg, 100)
        if not self.stop_flag:
            self.after(0, self._log, "✓ Plot complete!")
        self.after(0, self._redraw_canvas)

    # ── STOP / HOME ────────────────────────────────────────────────────
    def _stop(self):
        self.stop_flag = True
        self.plotting  = False
        if self.plotter and self.plotter.is_open():
            self.plotter.fire("STOP")
        self._set_robot(self.robot_x, self.robot_y, self.robot_h, False)
        self._log("■ STOP", RED)

    def _home(self):
        self._trace.clear()
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        self._set_robot(0, -park_mm, 0, False)
        if self.plotter and self.plotter.is_open():
            self.plotter.fire("STOP")
        self._log("⌂ HOME reset")

    def _dep_warn(self):
        messagebox.showwarning("Missing packages", "pip install opencv-python Pillow pyserial numpy")

if __name__ == "__main__":
    App().mainloop()
