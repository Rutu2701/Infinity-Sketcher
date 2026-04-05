"""Debug script to test skeleton path tracing with a circle."""
import cv2
import numpy as np
import sys

# Create a test image: just a circle
img = np.ones((200, 200), dtype=np.uint8) * 255
cv2.circle(img, (100, 100), 60, 0, 6)  # circle with thickness 6
cv2.imwrite("test_circle.png", img)

# Process like the actual pipeline
grey = img.copy()
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
eq = clahe.apply(grey)
blur = cv2.GaussianBlur(eq, (3, 3), 0)
otsu_val, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

cv2.imwrite("test_mask_circle.png", mask)

sys.path.insert(0, '.')
from v16 import extract_centerline, _prune_skeleton, _trace_skeleton_paths

# Test with different methods
for method in ['zhang_suen', 'distance']:
    print(f"\n=== Testing method: {method} ===")
    skel = extract_centerline(mask, method=method)
    cv2.imwrite(f"test_skel_{method}_circle.png", skel)
    
    skel_pixels = np.count_nonzero(skel > 0)
    print(f"Skeleton pixels: {skel_pixels}")
    
    # Check endpoints and junctions
    binary = (skel > 0).astype(np.uint8)
    k = np.ones((3, 3), dtype=np.uint8)
    neighbor_count = cv2.filter2D(binary, -1, k) - binary
    endpoints = (binary == 1) & (neighbor_count == 1)
    junctions = (binary == 1) & (neighbor_count >= 3)
    print(f"Endpoints: {np.count_nonzero(endpoints)}")
    print(f"Junctions: {np.count_nonzero(junctions)}")
    
    # Try tracing with different min_px
    for min_px in [1, 3, 5, 10]:
        strokes = _trace_skeleton_paths(skel, min_px=min_px)
        total_points = sum(len(s) for s in strokes)
        print(f"  min_px={min_px}: {len(strokes)} strokes, {total_points} total points")
        for i, s in enumerate(strokes[:5]):
            print(f"    Stroke {i}: {len(s)} points")

print("\nDone.")
