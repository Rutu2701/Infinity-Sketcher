"""
DIFF DRIVE PLOTTER - Potrace + GRBL Edition
Pipeline: CLAHE -> Otsu -> Potrace Vectorization -> GRBL G0/G1
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, time, os, math, subprocess, tempfile
import cv2, numpy as np
from PIL import Image, ImageTk, ImageDraw
import serial, serial.tools.list_ports
from svgelements import SVG, Path, Move, Line, Close, CubicBezier

HAS_DEPS = True

# ═══════════════════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════════════════
BAUD         = 115200
INTER_CMD_S  = 0.002
FULL_STEPS   = 200
MICROSTEPS   = 32

DEF_WHEEL_MM  = 60.0
DEF_AXLE_MM   = 120.0
DEF_DRAW_US   = 300
DEF_TRAVEL_US = 120
DEF_PAPER_W   = 150.0
DEF_PAPER_H   = 150.0
DEF_PARK_MM   = 20.0

DEF_SERVO_PIN  = 11
DEF_PEN_DOWN   = 30
DEF_PEN_UP     = 90

BG     = "#f5f0eb"; CARD   = "#ffffff"; CARD2  = "#ede8e2"
ACC    = "#b05a00"; ACC2   = "#7c3aed"; ACC3   = "#0369a1"
DIM    = "#6b7280"; WHITE  = "#1f2937"; GREEN  = "#166534"
RED    = "#dc2626"; AMBER  = "#d97706"

FONT_XL   = ("Courier", 14, "bold")
FONT_L    = ("Courier", 11, "bold")
FONT_M_B  = ("Courier", 10, "bold")
FONT_M    = ("Courier", 10)
FONT_S_B  = ("Courier", 9, "bold")
FONT_S    = ("Courier", 9)
FONT_XS   = ("Courier", 8)

# ═══════════════════════════════════════════════════════════════
#  KINEMATICS
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
    s = int(round(abs(deg) * STEPS_PER_DEG))
    if deg >= 0:
        return -s, +s
    else:
        return +s, -s

def resample(pts, step_mm):
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

def stroke_to_moves(pts, step_mm, is_draw, init_h):
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
        bear = math.degrees(math.atan2(dx, dy))
        if cur_h is None:
            cur_h = bear
        rot = (bear - cur_h + 180) % 360 - 180
        if abs(rot) > 0.5:
            rL, rR = rotate_steps(rot)
            if rL != 0 or rR != 0:
                moves.append((rL, rR, f"R{rot:+.1f}", is_draw))
            cur_h = bear
        steps = int(round(dist * STEPS_PER_MM))
        if steps > 0:
            moves.append((steps, steps, f"{'D' if is_draw else 'T'}{dist:.1f}", is_draw))
    return moves

def plan_moves(strokes, step_mm, travel_step_mm, park_mm=20.0):
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
            all_moves.append((m[0], m[1], m[2], False, pc))
            cur_x, cur_y, cur_h = pose_update(m[0], m[1], cur_x, cur_y, cur_h)

    for stroke in strokes:
        if len(stroke) < 2:
            continue
        sx, sy = stroke[0]
        travel_to(sx, sy)
        cur_x, cur_y = sx, sy
        dmoves = stroke_to_moves(stroke, step_mm, True, cur_h)
        for idx, m in enumerate(dmoves):
            pc = None
            if idx == 0 and not pen_is_down:
                pc = True
                pen_is_down = True
            all_moves.append((m[0], m[1], m[2], True, pc))
            cur_x, cur_y, cur_h = pose_update(m[0], m[1], cur_x, cur_y, cur_h)
        cur_x, cur_y = stroke[-1]

    travel_to(0.0, -park_mm)
    return all_moves

def simulate_moves(all_moves, park_mm=20.0):
    pts  = [(0.0, -park_mm, False)]
    x, y, h = 0.0, -park_mm, 0.0
    for stL, stR, lbl, is_draw, *_ in all_moves:
        x, y, h = pose_update(stL, stR, x, y, h)
        pts.append((x, y, is_draw))
    return pts

# ═══════════════════════════════════════════════════════════════
#  IMAGE PROCESSING
# ═══════════════════════════════════════════════════════════════

def auto_process_image(grey):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    eq    = clahe.apply(grey)
    blur  = cv2.GaussianBlur(eq, (3, 3), 0)
    otsu_val, mask = cv2.threshold(blur, 0, 255,
                                   cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    if np.mean(grey) > 200:
        pass
    elif np.mean(mask) > 128:
        mask = cv2.bitwise_not(mask)
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
    return mask, int(otsu_val)

def manual_process_image(grey, threshold, blur_r):
    if blur_r > 0:
        k    = max(3, blur_r * 2 + 1)
        grey = cv2.GaussianBlur(grey, (k, k), 0)
    if np.mean(grey) < 128:
        grey = cv2.bitwise_not(grey)
    _, mask = cv2.threshold(grey, threshold, 255, cv2.THRESH_BINARY_INV)
    return mask

def _parse_potrace_svg(svg_path, h):
    """Parse Potrace SVG output into list of stroke point lists."""
    with open(svg_path, 'r') as f:
        svg_content = f.read()
    svg = SVG.parse(svg_content)
    strokes = []
    for element in svg.elements():
        if isinstance(element, Path):
            points = []
            current_point = None
            for segment in element:
                if isinstance(segment, Move):
                    if points and len(points) > 1:
                        strokes.append(points)
                        points = []
                    current_point = (segment.end.x, segment.end.y)
                    points.append(current_point)
                elif isinstance(segment, (Line, Close)):
                    if current_point is not None:
                        current_point = (segment.end.x, segment.end.y)
                        points.append(current_point)
                elif isinstance(segment, CubicBezier):
                    if current_point is not None:
                        start = current_point
                        end = (segment.end.x, segment.end.y)
                        length = np.hypot(end[0]-start[0], end[1]-start[1])
                        num_samples = max(3, int(length / 5))
                        for i in range(1, num_samples + 1):
                            t = i / num_samples
                            x = (1-t)**3 * start[0] + 3*(1-t)**2*t * segment.control1.x + \
                                3*(1-t)*t**2 * segment.control2.x + t**3 * end[0]
                            y = (1-t)**3 * start[1] + 3*(1-t)**2*t * segment.control1.y + \
                                3*(1-t)*t**2 * segment.control2.y + t**3 * end[1]
                            points.append((x, y))
                        current_point = end
            if points and len(points) > 1:
                strokes.append(points)
    transformed = []
    for stroke in strokes:
        t = [(float(x), float(h - y)) for x, y in stroke]
        transformed.append(t)
    return transformed

def vectorize_with_potrace(skeleton_image, use_center=False):
    """Convert binary image to vector strokes using Potrace.
    
    Args:
        skeleton_image: Binary image (0/255)
        use_center: If True, use Potrace --center mode for centerline tracing.
                   If False, traces boundaries of filled regions.
    """
    if skeleton_image is None or skeleton_image.size == 0:
        return []
    if skeleton_image.dtype != np.uint8:
        skeleton_image = (skeleton_image > 127).astype(np.uint8) * 255
    h, w = skeleton_image.shape
    tmp_dir = tempfile.gettempdir()
    uid = str(os.getpid())
    pbm_path = os.path.join(tmp_dir, f"dd_{uid}.pbm")
    svg_path = os.path.join(tmp_dir, f"dd_{uid}.svg")
    try:
        if use_center:
            # Centerline mode: white lines on black background
            binary = skeleton_image.copy()
        else:
            # Boundary mode: invert so filled regions are white
            binary = 255 - skeleton_image
        ok = cv2.imwrite(pbm_path, binary)
        if not ok:
            return []
        cmd = ['potrace', pbm_path, '-s', '-o', svg_path,
               '--turdsize', '1', '--alphamax', '0.5', '--opttolerance', '0.2']
        if use_center:
            cmd.insert(2, '--center')
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=15,
            cwd=tmp_dir
        )
        if result.returncode != 0:
            return []
        if not os.path.exists(svg_path):
            return []
        return _parse_potrace_svg(svg_path, h)
    except Exception:
        return []
    finally:
        for p in [pbm_path, svg_path]:
            try:
                if os.path.exists(p):
                    os.unlink(p)
            except:
                pass

def _distance_transform_centerline(mask):
    """Extract centerline using distance transform + ridge detection.
    This finds the 'ridge' (local maxima) of the distance map,
    which corresponds to the center of strokes."""
    if mask is None or mask.size == 0:
        return np.zeros_like(mask)
    binary = (mask > 0).astype(np.uint8)
    dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    # Ridge = pixels that are local maxima in their neighborhood
    # Compare each pixel with its 8 neighbors
    p = np.pad(dist, 1, mode='constant')
    center = p[1:-1, 1:-1]
    neighbors = [
        p[0:-2, 0:-2], p[0:-2, 1:-1], p[0:-2, 2:],
        p[1:-1, 0:-2],                p[1:-1, 2:],
        p[2:, 0:-2],   p[2:, 1:-1],   p[2:, 2:]
    ]
    # Ridge: center >= all neighbors AND center > 0
    is_ridge = np.ones_like(center, dtype=bool)
    for n in neighbors:
        is_ridge &= (center >= n)
    is_ridge &= (center > 0)
    # Only keep ridge pixels that are part of the original shape
    ridge = (is_ridge & (binary > 0)).astype(np.uint8) * 255
    # Clean up: remove isolated pixels
    k = np.ones((3,3), np.uint8)
    ridge_cleaned = cv2.morphologyEx(ridge, cv2.MORPH_CLOSE, k, iterations=1)
    return ridge_cleaned

def _guo_hall_skeleton(mask):
    """Skeletonize using Guo-Hall algorithm (faster, cleaner than Zhang-Suen).
    Requires opencv-contrib-python."""
    try:
        import cv2.ximgproc
        binary = (mask > 0).astype(np.uint8) * 255
        skel = cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        return skel
    except (ImportError, AttributeError):
        return None

def _skimage_skeleton(mask):
    """Skeletonize using scikit-image's implementation."""
    try:
        from skimage.morphology import skeletonize
        binary = (mask > 0)
        skel = skeletonize(binary)
        return (skel.astype(np.uint8) * 255)
    except ImportError:
        return None

def extract_centerline(mask, method='auto'):
    """Extract 1-pixel centerline from binary mask using best available method.
    
    Args:
        mask: Binary image (0/255)
        method: 'auto', 'distance', 'guo_hall', 'skimage', 'zhang_suen'
    
    Returns:
        1-pixel wide skeleton (0/255)
    """
    if mask is None or mask.size == 0:
        return mask
    if method == 'auto':
        # Try methods in order of preference
        for m in ['guo_hall', 'skimage', 'distance', 'zhang_suen']:
            skel = extract_centerline(mask, m)
            if skel is not None and np.any(skel > 0):
                return skel
        return mask
    elif method == 'distance':
        return _distance_transform_centerline(mask)
    elif method == 'guo_hall':
        result = _guo_hall_skeleton(mask)
        return result if result is not None else None
    elif method == 'skimage':
        result = _skimage_skeleton(mask)
        return result if result is not None else None
    elif method == 'zhang_suen':
        return _zhang_suen(mask, max_iters=40)
    else:
        return _zhang_suen(mask, max_iters=40)

def _zhang_suen(binary_img, max_iters=40):
    if max_iters <= 0:
        return binary_img.copy()
    img = (binary_img > 0).astype(np.uint8)
    for _ in range(max_iters):
        changed = False
        for step in (0, 1):
            p  = np.pad(img, 1, mode='constant')
            P2 = p[0:-2,1:-1]; P3 = p[0:-2,2:  ]
            P4 = p[1:-1,2:  ]; P5 = p[2:  ,2:  ]
            P6 = p[2:  ,1:-1]; P7 = p[2:  ,0:-2]
            P8 = p[1:-1,0:-2]; P9 = p[0:-2,0:-2]
            N  = P2+P3+P4+P5+P6+P7+P8+P9
            ring = np.stack([P2,P3,P4,P5,P6,P7,P8,P9,P2], axis=2)
            T  = ((ring[:,:,:-1]==0)&(ring[:,:,1:]==1)).sum(axis=2)
            cond = (img==1)&(N>=2)&(N<=6)&(T==1)
            if step==0: mask = cond&((P2*P4*P6)==0)&((P4*P6*P8)==0)
            else:       mask = cond&((P2*P4*P8)==0)&((P2*P6*P8)==0)
            if mask.any():
                img[mask] = 0; changed = True
        if not changed:
            break
    return (img*255).astype(np.uint8)

def _prune_skeleton(skel_img, min_len=6):
    img = (skel_img > 0).astype(np.uint8)
    k   = np.ones((3,3), np.uint8)
    for _ in range(min_len):
        nbrs = cv2.filter2D(img, -1, k) - img
        tips = (img==1) & (nbrs==1)
        if not tips.any(): break
        img[tips] = 0
    return (img*255).astype(np.uint8)

def skeleton_to_strokes(skel, h, w, scale, off_x, off_y, min_px=20, eps_px=1.0):
    contours, _ = cv2.findContours(skel, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.arcLength(c, False) >= min_px]
    if not contours:
        return []
    MIN_MM  = max(0.5, scale * 0.8)
    def to_mm(px, py):
        return (round(float(px) * scale + off_x, 2),
                round(float(h - py) * scale + off_y, 2))
    strokes = []
    for cnt in contours:
        simp = cv2.approxPolyDP(cnt, eps_px, False).squeeze()
        if simp.ndim == 1: simp = simp[None,:]
        if len(simp) < 2:  continue
        mm, last = [], None
        for pt in simp:
            m = to_mm(float(pt[0]), float(pt[1]))
            if last is None or math.hypot(m[0]-last[0], m[1]-last[1]) >= MIN_MM:
                mm.append(m); last = m
        if len(mm) >= 2:
            strokes.append(mm)
    return strokes

def image_to_strokes(path, paper_w, paper_h,
                     auto=True, threshold=128, blur=1,
                     min_px=20, thin_res=256,
                     skeleton_method='auto',
                     progress_cb=None):
    def prog(m, p):
        if progress_cb: progress_cb(m, p)
    prog("Loading...", 5)
    img = cv2.imread(path)
    if img is None:
        raise ValueError(f"Cannot open: {path}")
    h, w = img.shape[:2]
    sc   = min(thin_res / w, thin_res / h)
    img  = cv2.resize(img, (max(1,int(w*sc)), max(1,int(h*sc))),
                      interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if auto:
        prog("Auto-contrast (CLAHE) + Otsu...", 18)
        mask, otsu_val = auto_process_image(grey)
        prog(f"Otsu threshold = {otsu_val}", 25)
    else:
        prog("Manual threshold...", 18)
        mask = manual_process_image(grey, threshold, blur)
    prog(f"Skeletonizing ({skeleton_method} method)...", 30)
    skel = extract_centerline(mask, method=skeleton_method)
    if skel is None or not np.any(skel > 0):
        prog("Centerline extraction failed, trying fallback...", 40)
        skel = extract_centerline(mask, method='zhang_suen')
    skel = _prune_skeleton(skel, min_len=6)
    scale = min(paper_w / w, paper_h / h)
    off_x = (paper_w - w * scale) / 2
    off_y = (paper_h - h * scale) / 2
    eps_px  = max(1.0, min(w,h) / 200.0)
    prog("Vectorizing skeleton with Potrace...", 50)
    strokes = vectorize_with_potrace(skel)
    if not strokes:
        prog("Potrace returned empty, using raw skeleton contours...", 70)
        strokes = skeleton_to_strokes(skel, h, w, scale, off_x, off_y, min_px, eps_px)
    if not strokes:
        raise ValueError("No strokes found - try adjusting threshold or use a clearer image.")
    prog("Scaling to mm...", 85)
    def to_mm(px, py):
        return (round(float(px) * scale + off_x, 2),
                round(float(h - py) * scale + off_y, 2))
    if strokes and isinstance(strokes[0][0], tuple) and len(strokes[0][0]) == 2:
        if isinstance(strokes[0][0][0], (int, float)):
            scaled_strokes = []
            for stroke in strokes:
                scaled = [to_mm(p[0], p[1]) for p in stroke]
                scaled_strokes.append(scaled)
            strokes = scaled_strokes
    prog(f"Done: {len(strokes)} strokes", 100)
    return strokes

def render_preview(sim_pts, paper_w, paper_h, W, H, park_mm=20.0, mirror_y=False):
    img  = Image.new("RGB", (W, H), (240, 238, 232))
    draw = ImageDraw.Draw(img)
    sx, sy = W / paper_w, H / paper_h
    def to_screen(x, y):
        return x * sx, (paper_h - y) * sy
    def clamp(px, py):
        return max(-2, min(W+2, px)), max(-2, min(H+2, py))
    prev = None
    for x, y, is_draw in sim_pts:
        if prev is not None:
            px0, py0 = clamp(*to_screen(*prev[:2]))
            px1, py1 = clamp(*to_screen(x, y))
            color = (160, 80, 0) if is_draw else (210, 218, 210)
            draw.line([int(px0), int(py0), int(px1), int(py1)], fill=color, width=1)
        prev = (x, y, is_draw)
    return img

# ═══════════════════════════════════════════════════════════════
#  SERIAL - GRBL G0/G1 Protocol
# ═══════════════════════════════════════════════════════════════

class PlotterSerial:
    def __init__(self):
        self.ser = None

    def ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baud=BAUD):
        self.ser = serial.Serial(port, baud, timeout=3)
        time.sleep(2)
        self.ser.reset_input_buffer()

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def fire(self, cmd):
        if not self.is_open():
            return
        try:
            self.ser.write((cmd.strip() + "\n").encode())
            self.ser.flush()
        except Exception:
            pass

    def wait_ok(self, timeout=10.0, stop_fn=None):
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            if stop_fn and stop_fn():
                raise InterruptedError("Stopped")
            if self.ser is None or not self.ser.is_open:
                raise IOError("Disconnected")
            try:
                n = self.ser.in_waiting
                if n:
                    buf += self.ser.read(n)
                    if b"OK"   in buf: return buf.decode(errors="replace").strip()
                    if b"ERR"  in buf: raise RuntimeError(buf.decode(errors="replace").strip())
                    if b"WARN" in buf: return buf.decode(errors="replace").strip()
            except (IOError, OSError) as e:
                raise IOError(str(e))
            time.sleep(0.004)
        raise TimeoutError(f"No OK after {timeout:.0f}s")

    def ping(self):
        if not self.is_open():
            return False
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"PING\n")
            self.wait_ok(timeout=2.0)
            return True
        except Exception:
            return False

    def send_steps(self, stL, stR, timeout=10.0, stop_fn=None):
        if not self.is_open():
            raise IOError("Not connected")
        self.ser.reset_input_buffer()
        self.ser.write(f"STEPS L:{stL} R:{stR}\n".encode())
        return self.wait_ok(timeout=timeout, stop_fn=stop_fn)

    def send_pen(self, down):
        self.fire("PEN DOWN" if down else "PEN UP")

    def send_travel_speed(self, us):
        self.fire(f"TSPEED {us}")

def cmd_timeout(stL, stR, step_delay_us):
    total    = max(abs(stL), abs(stR))
    if total == 0:
        return 2.0
    start_us = max(1500, step_delay_us * 5)
    ramp     = min(60, total // 2)
    avg_ramp = (start_us + step_delay_us) / 2.0
    t = (ramp * avg_ramp * 2 + max(0, total - 2*ramp) * step_delay_us) / 1e6
    return max(5.0, t * 3.0)

# ═══════════════════════════════════════════════════════════════
#  APP
# ═══════════════════════════════════════════════════════════════

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Diff Drive Plotter  Potrace+GRBL")
        self.configure(bg=BG)
        self.minsize(1150, 650)
        self.plotter    = PlotterSerial() if HAS_DEPS else None
        self.strokes    = []
        self.all_moves  = []
        self.sim_pts    = []
        self.image_path = None
        self.plotting   = False
        self.stop_flag  = False
        self._tk_img    = None
        self._trace     = []
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_h    = 0.0
        self.pen_down   = False
        self._img_aspect = 1.0
        self._running   = True
        self._tick_id   = None
        self.port_var  = tk.StringVar()
        self.baud_var  = tk.StringVar(value="115200")
        self.conn_var  = tk.StringVar(value="CONNECT")
        self.stat_var  = tk.StringVar(value="OFFLINE")
        self.out_w     = tk.DoubleVar(value=DEF_PAPER_W)
        self.out_h     = tk.DoubleVar(value=DEF_PAPER_H)
        self.lock_ar   = tk.BooleanVar(value=True)
        self.prog_var  = tk.DoubleVar(value=0)
        self.prog_lbl  = tk.StringVar(value="idle")
        self.s_thr    = tk.IntVar(value=128)
        self.s_blur   = tk.IntVar(value=1)
        self.s_min    = tk.IntVar(value=20)
        self.s_thin   = tk.IntVar(value=256)
        self.s_auto   = tk.BooleanVar(value=True)
        self.s_step   = tk.IntVar(value=10)
        self.s_travel = tk.IntVar(value=50)
        self.e_wheel      = tk.StringVar(value=str(int(DEF_WHEEL_MM)))
        self.e_axle       = tk.StringVar(value=str(int(DEF_AXLE_MM)))
        self.e_speed      = tk.StringVar(value=str(DEF_DRAW_US))
        self.e_tspeed     = tk.StringVar(value=str(DEF_TRAVEL_US))
        self.e_park       = tk.StringVar(value=str(int(DEF_PARK_MM)))
        self.e_servo_pin  = tk.StringVar(value=str(DEF_SERVO_PIN))
        self.e_pen_down   = tk.StringVar(value=str(DEF_PEN_DOWN))
        self.e_pen_up     = tk.StringVar(value=str(DEF_PEN_UP))
        self.s_speed  = tk.IntVar(value=DEF_DRAW_US)
        self.s_mirror = tk.BooleanVar(value=True)
        self.s_skel_method = tk.StringVar(value="auto")
        self._build()
        self.bind("<F11>", self._toggle_fs)
        self._tick_id = self.after(300, self._refresh_ports)
        self._tick_id = self.after(80, self._tick)
        if not HAS_DEPS:
            self.after(600, self._dep_warn)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        self._running = False
        self.stop_flag = True
        if self.plotter and self.plotter.is_open():
            self.plotter.fire("STOP")
            self.plotter.disconnect()
        if self._tick_id:
            self.after_cancel(self._tick_id)
        self.destroy()

    def _build(self):
        top = tk.Frame(self, bg=CARD2, pady=6)
        top.pack(fill="x", side="top")
        tk.Label(top, text="DIFF PLOTTER  Potrace+GRBL", bg=CARD2, fg=ACC,
                 font=FONT_XL, padx=14).pack(side="left")
        for txt, cmd, fg in [
            ("GENERATE",  self._generate,   ACC2),
            ("DEMO",      self._demo_run,   "#6366f1"),
            ("START",     self._plot,        GREEN),
            ("STOP",      self._stop,        RED),
            ("HOME",      self._home,        DIM),
        ]:
            tk.Button(top, text=txt, command=cmd, bg=CARD, fg=fg,
                      font=FONT_L, bd=0, padx=10, pady=4,
                      cursor="hand2", activebackground=BG, activeforeground=fg
                      ).pack(side="left", padx=3)
        conn_fr = tk.Frame(top, bg=CARD2)
        conn_fr.pack(side="right", padx=10)
        self._port_cb = ttk.Combobox(conn_fr, textvariable=self.port_var,
                                     width=12, font=FONT_M, state="readonly")
        self._port_cb.pack(side="left", padx=2)
        tk.Button(conn_fr, text="REFRESH", bg=CARD2, fg=ACC2, bd=0,
                  font=FONT_M, cursor="hand2", command=self._refresh_ports).pack(side="left")
        ttk.Combobox(conn_fr, textvariable=self.baud_var, width=7,
                     values=["9600","57600","115200"],
                     font=FONT_M, state="readonly").pack(side="left", padx=2)
        tk.Button(conn_fr, textvariable=self.conn_var, command=self._toggle_conn,
                  bg=CARD, fg=ACC3, font=FONT_L, bd=0, padx=8, pady=4,
                  cursor="hand2").pack(side="left", padx=4)
        self._stat_lbl = tk.Label(conn_fr, textvariable=self.stat_var,
                                   bg=CARD2, fg=RED, font=FONT_M_B)
        self._stat_lbl.pack(side="left", padx=4)
        pf = tk.Frame(self, bg=CARD2)
        pf.pack(fill="x", side="top")
        ttk.Progressbar(pf, variable=self.prog_var, maximum=100,
                        length=600).pack(side="left", fill="x", expand=True, padx=8, pady=2)
        tk.Label(pf, textvariable=self.prog_lbl, bg=CARD2, fg=DIM, font=FONT_S).pack(side="left", padx=4)
        main = tk.Frame(self, bg=BG)
        main.pack(fill="both", expand=True)
        main.columnconfigure(1, weight=1)
        main.columnconfigure(2, weight=1)
        main.rowconfigure(0, weight=1)
        lf_container = tk.Frame(main, bg=CARD, width=320)
        lf_container.grid(row=0, column=0, sticky="nsew", padx=(6,3), pady=6)
        lf_container.grid_propagate(False)
        self.lf_canvas = tk.Canvas(lf_container, bg=CARD, highlightthickness=0)
        self.lf_scroll = ttk.Scrollbar(lf_container, orient="vertical", command=self.lf_canvas.yview)
        self.lf = tk.Frame(self.lf_canvas, bg=CARD)
        self.lf.bind("<Configure>", lambda e: self.lf_canvas.configure(scrollregion=self.lf_canvas.bbox("all")))
        self.lf_canvas.create_window((0, 0), window=self.lf, anchor="nw", width=300)
        self.lf_canvas.configure(yscrollcommand=self.lf_scroll.set)
        self.lf_canvas.pack(side="left", fill="both", expand=True)
        self.lf_scroll.pack(side="right", fill="y")
        def _on_mousewheel(event):
            if event.num == 4:
                self.lf_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self.lf_canvas.yview_scroll(1, "units")
            else:
                self.lf_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        self.bind_all("<MouseWheel>", _on_mousewheel)
        self.bind_all("<Button-4>", _on_mousewheel)
        self.bind_all("<Button-5>", _on_mousewheel)
        self._build_left(self.lf)
        cf = tk.Frame(main, bg=CARD)
        cf.grid(row=0, column=1, sticky="nsew", padx=3, pady=6)
        self._build_centre(cf)
        rf = tk.Frame(main, bg=CARD)
        rf.grid(row=0, column=2, sticky="nsew", padx=(3,6), pady=6)
        self._build_right(rf)
        lf2 = tk.Frame(self, bg=CARD2)
        lf2.pack(fill="x", side="bottom")
        tk.Label(lf2, text="LOG", bg=CARD2, fg=DIM, font=FONT_S_B, pady=2, padx=8).pack(anchor="w")
        self.log_txt = tk.Text(lf2, height=3, bg=CARD, fg=ACC3, font=FONT_S, bd=0, state="disabled", wrap="word")
        self.log_txt.pack(fill="x", padx=6, pady=(0, 4))

    def _build_left(self, p):
        def sec(t):
            tk.Label(p, text=t, bg=CARD, fg=ACC, font=FONT_M_B,
                     anchor="w", pady=4).pack(fill="x", padx=8)
        def sldr(lbl, var, lo, hi, fmt=None, res=1.0, widgets_ref=None):
            row = tk.Frame(p, bg=CARD); row.pack(fill="x", padx=8)
            disp = tk.StringVar()
            def upd(*a):
                try: v=var.get(); disp.set(fmt(v) if fmt else str(v))
                except: pass
            var.trace_add("write", upd); upd()
            lw = tk.Label(row, text=lbl, bg=CARD, fg=DIM, font=FONT_S,
                          anchor="w", width=14)
            lw.pack(side="left")
            dw = tk.Label(row, textvariable=disp, bg=CARD, fg=ACC,
                          font=FONT_S_B, width=6, anchor="e")
            dw.pack(side="right")
            sc = tk.Scale(p, variable=var, from_=lo, to=hi,
                          orient="horizontal", resolution=res,
                          bg=CARD, fg=ACC, troughcolor=CARD2,
                          highlightthickness=0, sliderlength=15,
                          showvalue=False, bd=0)
            sc.pack(fill="x", padx=8, pady=0)
            if widgets_ref is not None:
                widgets_ref.extend([lw, dw, sc])
            return sc
        def entry_row(lbl, var, unit="", step=1.0):
            row = tk.Frame(p, bg=CARD); row.pack(fill="x", padx=8, pady=2)
            tk.Label(row, text=lbl, bg=CARD, fg=DIM, font=FONT_S,
                     width=16, anchor="w").pack(side="left")
            e = tk.Entry(row, textvariable=var, bg=CARD2, fg=WHITE,
                         font=FONT_M, width=7, bd=0, insertbackground=WHITE)
            e.pack(side="left", padx=4)
            if unit:
                tk.Label(row, text=unit, bg=CARD, fg=DIM,
                         font=FONT_S).pack(side="left")
            def _scroll(event, _var=var, _step=step):
                try:
                    delta = -1 if (event.delta > 0 or event.num == 4) else 1
                    val   = float(_var.get()) + delta * _step
                    _var.set(str(round(val, 4)))
                except Exception:
                    pass
            e.bind("<MouseWheel>", _scroll)
            e.bind("<Button-4>",   _scroll)
            e.bind("<Button-5>",   _scroll)
        sec("01  IMAGE")
        tk.Button(p, text="UPLOAD IMAGE", command=self._upload,
                  bg=CARD2, fg=ACC, font=FONT_M_B, bd=0,
                  cursor="hand2", pady=5).pack(fill="x", padx=8, pady=(0,2))
        self.lbl_file = tk.Label(p, text="no file", bg=CARD, fg=DIM,
                                  font=FONT_S, wraplength=200)
        self.lbl_file.pack(anchor="w", padx=8)
        auto_row = tk.Frame(p, bg=CARD2, pady=3)
        auto_row.pack(fill="x", padx=8, pady=(4,0))
        tk.Checkbutton(auto_row, text="AUTO  (CLAHE + Otsu)",
                       variable=self.s_auto, bg=CARD2, fg=ACC,
                       activebackground=CARD2, selectcolor=CARD,
                       font=FONT_M_B, command=self._on_auto_toggle
                       ).pack(side="left")
        self.lbl_otsu = tk.Label(auto_row, text="", bg=CARD2,
                                  fg=DIM, font=FONT_XS)
        self.lbl_otsu.pack(side="right", padx=4)
        self._manual_widgets = []
        sldr("Threshold",   self.s_thr,  50, 250, widgets_ref=self._manual_widgets)
        sldr("Blur",        self.s_blur,  0,   5,  widgets_ref=self._manual_widgets)
        sldr("Min stroke",  self.s_min,   5, 100)
        skel_row = tk.Frame(p, bg=CARD); skel_row.pack(fill="x", padx=8, pady=4)
        tk.Label(skel_row, text="Skeleton method", bg=CARD, fg=DIM,
                 font=FONT_S, width=14, anchor="w").pack(side="left")
        skel_cb = ttk.Combobox(skel_row, textvariable=self.s_skel_method,
                               width=10, font=FONT_S, state="readonly",
                               values=["auto", "guo_hall", "skimage", "distance", "zhang_suen"])
        skel_cb.pack(side="left")
        skel_cb.set("auto")
        tk.Frame(p, bg=CARD2, height=1).pack(fill="x", padx=8, pady=4)
        self._on_auto_toggle()
        sec("02  OUTPUT SIZE (mm)")
        for lbl, var in [("W", self.out_w), ("H", self.out_h)]:
            row = tk.Frame(p, bg=CARD); row.pack(fill="x", padx=8, pady=1)
            tk.Label(row, text=lbl, bg=CARD, fg=DIM, font=FONT_M,
                     width=2).pack(side="left")
            e = tk.Entry(row, textvariable=var, bg=CARD2, fg=WHITE,
                         font=FONT_M, width=6, bd=0, insertbackground=WHITE)
            e.pack(side="left", padx=4)
            tk.Label(row, text="mm", bg=CARD, fg=DIM,
                     font=FONT_M).pack(side="left")
        tk.Checkbutton(p, text="Lock ratio", variable=self.lock_ar,
                       bg=CARD, fg=DIM, activebackground=CARD,
                       selectcolor=CARD2, font=FONT_S).pack(anchor="w", padx=8)
        pr = tk.Frame(p, bg=CARD); pr.pack(fill="x", padx=8, pady=2)
        for lbl, w, h in [("100",100,100),("150",150,150),
                          ("A5",148,210),("A4",210,297),("US-L",216,279)]:
            tk.Button(pr, text=lbl,
                      command=lambda ww=w,hh=h: self._set_size(ww,hh),
                      bg=CARD2, fg=DIM, font=FONT_XS, bd=0,
                      cursor="hand2", padx=2, pady=2).pack(side="left", padx=1)
        tk.Frame(p, bg=CARD2, height=1).pack(fill="x", padx=8, pady=4)
        sec("03  PATH")
        sldr("Draw step (x0.1)", self.s_step,   5,  30,
             fmt=lambda v: f"{v/10:.1f}mm")
        sldr("Travel step",      self.s_travel, 10, 100,
             fmt=lambda v: f"{v/10:.1f}mm")
        tk.Checkbutton(p, text="Fix Physical Mirror (Flip Y)",
                       variable=self.s_mirror, bg=CARD, fg=DIM,
                       activebackground=CARD, selectcolor=CARD2,
                       font=FONT_S, command=self._do_plan
                       ).pack(anchor="w", padx=8, pady=2)
        tk.Frame(p, bg=CARD2, height=1).pack(fill="x", padx=8, pady=4)
        sec("04  ROBOT GEOMETRY")
        entry_row("Wheel dia",    self.e_wheel,  "mm", step=1.0)
        entry_row("Axle width",   self.e_axle,   "mm", step=1.0)
        entry_row("Draw speed",   self.e_speed,  "us", step=10.0)
        entry_row("Travel speed", self.e_tspeed, "us", step=10.0)
        entry_row("Park offset",  self.e_park,   "mm", step=5.0)
        tk.Button(p, text="UPDATE PARAMS", command=self._update_params,
                  bg=CARD2, fg=ACC3, font=FONT_M_B, bd=0,
                  cursor="hand2", pady=5).pack(fill="x", padx=8, pady=4)
        tk.Frame(p, bg=CARD2, height=1).pack(fill="x", padx=8, pady=2)
        sec("05  SERVO / PEN")
        entry_row("Servo pin",   self.e_servo_pin, "",  step=1.0)
        entry_row("Pen-down deg",  self.e_pen_down,  "deg", step=5.0)
        entry_row("Pen-up deg",    self.e_pen_up,    "deg", step=5.0)
        tk.Button(p, text="SEND SERVO CFG", command=self._send_servo_cfg,
                  bg=CARD2, fg=ACC3, font=FONT_M_B, bd=0,
                  cursor="hand2", pady=4).pack(fill="x", padx=8, pady=4)
        tk.Frame(p, bg=CARD2, height=1).pack(fill="x", padx=8, pady=2)
        self.lbl_stat = tk.Label(p, text="", bg=CARD, fg=DIM,
                                  font=FONT_S, justify="left")
        self.lbl_stat.pack(anchor="w", padx=8, pady=2)

    def _on_auto_toggle(self):
        state = "disabled" if self.s_auto.get() else "normal"
        fg    = CARD2 if self.s_auto.get() else DIM
        for w in self._manual_widgets:
            try:
                w.configure(state=state)
                if hasattr(w, 'configure'):
                    try: w.configure(fg=fg)
                    except: pass
            except Exception:
                pass

    def _build_centre(self, p):
        p.columnconfigure(0, weight=1)
        p.rowconfigure(1, weight=1)
        tk.Label(p, text="PATH PREVIEW  (actual robot path)", bg=CARD, fg=ACC, font=FONT_M_B, pady=4).grid(row=0, column=0, sticky="ew")
        self.prev_cv = tk.Canvas(p, bg="#e8e6e0", bd=0, highlightthickness=0)
        self.prev_cv.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))
        self.prev_cv.bind("<Configure>", lambda e: self._redraw_preview())

    def _build_right(self, p):
        p.columnconfigure(0, weight=1)
        p.rowconfigure(1, weight=1)
        tk.Label(p, text="LIVE ROBOT VIEW", bg=CARD, fg=ACC3, font=FONT_M_B, pady=4).grid(row=0, column=0, sticky="ew")
        self.live_cv = tk.Canvas(p, bg="#e8e6e0", bd=0, highlightthickness=0)
        self.live_cv.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))

    def _log(self, msg, color=ACC3):
        self.log_txt.configure(state="normal")
        self.log_txt.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_txt.see("end")
        self.log_txt.configure(state="disabled")

    def _prog(self, msg, pct):
        self.prog_var.set(pct)
        self.prog_lbl.set(msg)

    def _toggle_fs(self, e=None):
        fs = not self.attributes("-fullscreen")
        self.attributes("-fullscreen", fs)

    def _refresh_ports(self):
        if not HAS_DEPS or not self._running:
            return
        ports = self.plotter.ports()
        self._port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if self._running:
            self._tick_id = self.after(2000, self._refresh_ports)

    def _set_size(self, w, h):
        self.out_w.set(float(w))
        if self.lock_ar.get() and self._img_aspect > 0:
            self.out_h.set(round(w / self._img_aspect, 1))
        else:
            self.out_h.set(float(h))

    def _update_params(self):
        try:
            wheel = float(self.e_wheel.get())
            axle  = float(self.e_axle.get())
            speed = int(self.e_speed.get())
            tspeed = int(self.e_tspeed.get())
        except ValueError:
            messagebox.showerror("Bad value", "Wheel/Axle/Speed must be numbers.")
            return
        update_geometry(wheel, axle)
        self.s_speed.set(speed)
        self._log(f"Wheel={WHEEL_MM}mm Axle={AXLE_MM}mm SPmm={STEPS_PER_MM:.2f} Draw={speed}us Travel={tspeed}us")
        if self.plotter and self.plotter.is_open():
            self.plotter.fire(f"SPEED {speed}")
            self.plotter.send_travel_speed(tspeed)
        if self.strokes:
            self._do_plan()
            self._redraw_preview()

    def _send_servo_cfg(self):
        if not (self.plotter and self.plotter.is_open()):
            messagebox.showwarning("Offline", "Connect to Arduino first.")
            return
        try:
            pin  = int(self.e_servo_pin.get())
            down = int(self.e_pen_down.get())
            up   = int(self.e_pen_up.get())
        except ValueError:
            messagebox.showerror("Bad value", "Servo pin/angles must be integers.")
            return
        cmd = f"SERVO {pin} {down} {up}"
        self.plotter.fire(cmd)
        self._log(f"Sent: {cmd}")

    def _toggle_conn(self):
        if not HAS_DEPS: self._dep_warn(); return
        if self.plotter.is_open():
            self.plotter.disconnect()
            self.conn_var.set("CONNECT")
            self.stat_var.set("OFFLINE")
            self._stat_lbl.configure(fg=RED)
            self._log("Disconnected")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return
        try:
            self.plotter.connect(port, int(self.baud_var.get()))
            self.conn_var.set("DISCONNECT")
            self.stat_var.set(f"ONLINE {port}")
            self._stat_lbl.configure(fg=GREEN)
            self._log(f"Connected {port} @ {self.baud_var.get()}")
            if self.plotter.ping():
                self._log("PING OK")
                self.plotter.fire(f"SPEED {self.s_speed.get()}")
                try:
                    self.plotter.send_travel_speed(int(self.e_tspeed.get()))
                    pin  = int(self.e_servo_pin.get())
                    down = int(self.e_pen_down.get())
                    up   = int(self.e_pen_up.get())
                    self.plotter.fire(f"SERVO {pin} {down} {up}")
                except Exception:
                    pass
            else:
                self._log("No PING - wrong firmware?", AMBER)
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))
            self._log(f"ERROR: {e}", RED)

    def _upload(self):
        path = filedialog.askopenfilename(
            title="Select image",
            filetypes=[("Images","*.png *.jpg *.jpeg *.bmp *.gif *.tiff"), ("All","*.*")])
        if not path: return
        self.image_path = path
        self.lbl_file.configure(text=os.path.basename(path)[:28], fg=WHITE)
        try:
            from PIL import Image as _I
            with _I.open(path) as im:
                iw, ih = im.size
                self._img_aspect = iw / ih if ih > 0 else 1.0
        except Exception:
            pass
        self._generate()

    def _generate(self):
        if not HAS_DEPS: self._dep_warn(); return
        if not self.image_path:
            messagebox.showwarning("No image", "Upload an image first.")
            return
        if self.plotting: return
        self._prog("Converting...", 5)
        threading.Thread(target=self._gen_thread, daemon=True).start()

    def _gen_thread(self):
        try:
            pw, ph = float(self.out_w.get()), float(self.out_h.get())
        except Exception:
            pw, ph = DEF_PAPER_W, DEF_PAPER_H
        try:
            strokes = image_to_strokes(
                self.image_path, pw, ph,
                auto            = self.s_auto.get(),
                threshold       = self.s_thr.get(),
                blur            = self.s_blur.get(),
                min_px          = self.s_min.get(),
                thin_res        = self.s_thin.get(),
                skeleton_method = self.s_skel_method.get(),
                progress_cb     = lambda m, p: self.after(0, self._prog, m, p))
            self.strokes = strokes
            self.after(0, self._do_plan)
        except Exception as e:
            self.after(0, messagebox.showerror, "Convert error", str(e))
            self.after(0, self._prog, f"Error: {e}", 0)

    def _do_plan(self):
        if not self.strokes: return
        step_mm   = max(0.5, self.s_step.get() / 10.0)
        travel_mm = max(1.0, self.s_travel.get() / 10.0)
        try:
            update_geometry(float(self.e_wheel.get()), float(self.e_axle.get()))
        except ValueError:
            pass
        strokes_to_plan = self.strokes
        if self.s_mirror.get():
            strokes_to_plan = [[(x, -y) for x, y in s] for s in self.strokes]
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except ValueError:
            park_mm = DEF_PARK_MM
        self.all_moves = plan_moves(strokes_to_plan, step_mm, travel_mm, park_mm)
        self.sim_pts   = simulate_moves(self.all_moves, park_mm)
        nd = sum(1 for m in self.all_moves if m[3])
        self.lbl_stat.configure(text=f"Strokes: {len(self.strokes)}\n"
                                     f"Moves: {len(self.all_moves)} ({nd} draw)")
        self._log(f"{len(self.strokes)} strokes, {len(self.all_moves)} moves")
        self._redraw_preview()

    def _redraw_preview(self):
        if not self.sim_pts: return
        cw = max(self.prev_cv.winfo_width(),  80)
        ch = max(self.prev_cv.winfo_height(), 80)
        try:
            pw, ph = float(self.out_w.get()), float(self.out_h.get())
        except Exception:
            pw, ph = DEF_PAPER_W, DEF_PAPER_H
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        asp = pw / ph if ph > 0 else 1.0
        if cw / ch > asp: rh, rw = ch, int(ch * asp)
        else:             rw, rh = cw, int(cw / asp)
        rw, rh = max(rw, 10), max(rh, 10)
        threading.Thread(target=self._render_thread,
                         args=(rw, rh, pw, ph, park_mm), daemon=True).start()

    def _render_thread(self, W, H, pw, ph, park_mm):
        pil    = render_preview(self.sim_pts, pw, ph, W, H, park_mm)
        tk_img = ImageTk.PhotoImage(pil)
        self._tk_img = tk_img
        self.after(0, self._blit_preview, tk_img)

    def _blit_preview(self, img):
        self.prev_cv.delete("all")
        self.prev_cv.create_image(0, 0, anchor="nw", image=img)

    def _tick(self):
        if not self._running:
            return
        self._draw_live()
        self._tick_id = self.after(50, self._tick)

    def _draw_live(self):
        c = self.live_cv
        c.delete("all")
        cw, ch = c.winfo_width(), c.winfo_height()
        if cw < 20 or ch < 20: return
        try:
            pw, ph = float(self.out_w.get()), float(self.out_h.get())
        except Exception:
            pw, ph = DEF_PAPER_W, DEF_PAPER_H
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        all_x = [self.robot_x] + [x for x, y in self._trace]
        all_y = [self.robot_y] + [y for x, y in self._trace]
        vx_min = min(0.0,       min(all_x) - 5)
        vx_max = max(pw,        max(all_x) + 5)
        vy_min = min(-park_mm,  min(all_y) - 5)
        vy_max = max(ph,        max(all_y) + 5)
        view_w = vx_max - vx_min
        view_h = vy_max - vy_min
        ml, mr, mt, mb = 36, 8, 8, 24
        aw = cw - ml - mr
        ah = ch - mt - mb
        if aw < 10 or ah < 10: return
        sc  = min(aw / view_w, ah / view_h)
        ox  = ml - vx_min * sc
        oy  = mt + vy_max * sc
        def to_sx(xr): return ox + xr * sc
        def to_sy(yr): return oy - yr * sc
        c.create_rectangle(0, 0, cw, ch, fill="#dde8dc", outline="")
        pkx0, pky0 = to_sx(0),  to_sy(0)
        pkx1, pky1 = to_sx(pw), to_sy(-park_mm)
        c.create_rectangle(pkx0, pky0, pkx1, pky1,
                            fill="#e8f0e8", outline="#aaccaa", width=1, dash=(3,3))
        c.create_text((pkx0+pkx1)//2, (pky0+pky1)//2,
                      text="PARK", fill="#aaccaa", font=FONT_XS)
        px0, py0 = to_sx(0),  to_sy(ph)
        px1, py1 = to_sx(pw), to_sy(0)
        c.create_rectangle(px0, py0, px1, py1,
                            fill="#f0eeea", outline="#9aaa90", width=1)
        grid = 25 if pw <= 200 else 50
        for xm in range(0, int(pw)+1, grid):
            gx = to_sx(xm)
            c.create_line(gx, py0, gx, py1, fill="#ccdcc8", width=1)
            if xm > 0: c.create_text(gx, py1+8, text=str(xm), fill=DIM, font=FONT_S)
        for ym in range(0, int(ph)+1, grid):
            gy = to_sy(ym)
            c.create_line(px0, gy, px1, gy, fill="#ccdcc8", width=1)
            if ym > 0: c.create_text(px0-4, gy, text=str(ym),
                                     fill=DIM, font=FONT_S, anchor="e")
        if len(self._trace) >= 2:
            flat = []
            for tx, ty in self._trace:
                flat.extend([to_sx(tx), to_sy(ty)])
            if len(flat) >= 4:
                c.create_line(flat, fill=ACC, width=1)
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
                            f"  H:{self.robot_h:.1f}deg"
                            f"  {'DRAW' if self.pen_down else 'TRAVEL'}"),
                      fill=ACC3 if self.pen_down else DIM, font=FONT_S_B)

    def _set_robot(self, x, y, h, pen):
        self.robot_x  = x
        self.robot_y  = y
        self.robot_h  = h
        self.pen_down = pen
        if pen:
            self._trace.append((x, y))

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
            stL, stR, lbl, is_draw = move[0], move[1], move[2], move[3]
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
                   "Demo done" if not self.stop_flag else "Stopped", 100)

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
        self.after(0, self._log, f"START {total} moves  draw={spd}us travel={tspd}us")
        cur_x, cur_y = 0.0, -park_mm
        for i, move in enumerate(self.all_moves):
            if self.stop_flag: break
            stL, stR, lbl, is_draw, pen_change = \
                move[0], move[1], move[2], move[3], move[4]
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
                self.after(0, self._log, f"Disconnected: {e}", RED); break
            except TimeoutError as e:
                self.after(0, self._log, f"TIMEOUT move {i+1}: {e}", RED)
                errs += 1
                if errs >= 3:
                    self.after(0, messagebox.showerror, "3 Timeouts",
                               "Arduino not responding.\nCheck 12V + Vref.")
                    break
            except RuntimeError as e:
                self.after(0, self._log, f"Arduino ERR: {e}", RED)
                self.after(0, messagebox.showerror, "Arduino Error", str(e))
                break
            except Exception as e:
                self.after(0, self._log, f"Error: {e}", RED); break
            else:
                time.sleep(INTER_CMD_S)
        if self.plotter and self.plotter.is_open():
            self.plotter.send_pen(False)
        self.plotting = False
        msg = "Done" if not self.stop_flag else "Stopped"
        self.after(0, self._prog, msg, 100)
        if not self.stop_flag:
            self.after(0, self._log, "Plot complete!")

    def _stop(self):
        self.stop_flag = True
        self.plotting  = False
        if self.plotter and self.plotter.is_open():
            self.plotter.fire("STOP")
        self._set_robot(self.robot_x, self.robot_y, self.robot_h, False)
        self._log("STOP", RED)

    def _home(self):
        self._trace.clear()
        try:
            park_mm = max(5.0, float(self.e_park.get()))
        except Exception:
            park_mm = DEF_PARK_MM
        self._set_robot(0, -park_mm, 0, False)
        if self.plotter and self.plotter.is_open():
            self.plotter.fire("STOP")
        self._log("HOME reset")

    def _dep_warn(self):
        messagebox.showwarning("Missing packages",
            "pip install opencv-python Pillow pyserial numpy svgelements\n"
            "sudo apt install python3-pil.imagetk potrace")

if __name__ == "__main__":
    App().mainloop()
