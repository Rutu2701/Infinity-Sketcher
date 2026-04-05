"""Debug script to test skeleton path tracing with a more complex image."""
import cv2
import numpy as np
import sys

# Create a test image that looks like a real drawing
img = np.ones((300, 300), dtype=np.uint8) * 255

# Draw some thick lines like a real sketch
cv2.line(img, (30, 30), (270, 30), 0, 6)  # top
cv2.line(img, (270, 30), (270, 270), 0, 6)  # right
cv2.line(img, (270, 270), (30, 270), 0, 6)  # bottom
cv2.line(img, (30, 270), (30, 30), 0, 6)  # left
cv2.circle(img, (150, 150), 50, 0, 5)  # circle in middle
cv2.line(img, (30, 30), (270, 270), 0, 4)  # diagonal

cv2.imwrite("test_complex.png", img)

# Process like the actual pipeline
grey = img.copy()
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
eq = clahe.apply(grey)
blur = cv2.GaussianBlur(eq, (3, 3), 0)
otsu_val, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

cv2.imwrite("test_mask.png", mask)

sys.path.insert(0, '.')
from v16 import extract_centerline, _prune_skeleton, _trace_skeleton_paths

for method in ['zhang_suen', 'distance']:
    print(f"\n=== Testing method: {method} ===")
    skel = extract_centerline(mask, method=method)
    cv2.imwrite(f"test_skel_{method}.png", skel)
    
    skel_pixels = np.count_nonzero(skel > 0)
    print(f"Skeleton pixels: {skel_pixels}")
    
    skel_pruned = _prune_skeleton(skel, min_len=3)
    cv2.imwrite(f"test_skel_{method}_pruned.png", skel_pruned)
    skel_pruned_pixels = np.count_nonzero(skel_pruned > 0)
    print(f"After pruning: {skel_pruned_pixels} pixels")
    
    # Try different min_px values
    for min_px in [1, 5, 10, 20]:
        strokes = _trace_skeleton_paths(skel_pruned, min_px=min_px)
        total_points = sum(len(s) for s in strokes)
        print(f"  min_px={min_px}: {len(strokes)} strokes, {total_points} total points")
        for i, s in enumerate(strokes[:5]):
            print(f"    Stroke {i}: {len(s)} points")

print("\nDone.")
