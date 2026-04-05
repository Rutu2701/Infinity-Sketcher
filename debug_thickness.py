"""Debug script to test skeleton thickness."""
import cv2
import numpy as np
import sys

# Create a simple test: just a circle
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
from v16 import extract_centerline, _thin_to_1px

# Test with different methods
for method in ['zhang_suen', 'distance']:
    print(f"\n=== Testing method: {method} ===")
    skel = extract_centerline(mask, method=method)
    
    # Count pixels that have >2 neighbors (indicates thickness)
    binary = (skel > 0).astype(np.uint8)
    k = np.ones((3, 3), dtype=np.uint8)
    neighbor_count = cv2.filter2D(binary, -1, k) - binary
    thick_pixels = np.count_nonzero((binary == 1) & (neighbor_count > 2))
    print(f"Before thinning: {np.count_nonzero(skel > 0)} pixels, {thick_pixels} thick pixels")
    
    # Apply thinning
    skel_thinned = _thin_to_1px(skel)
    binary_thinned = (skel_thinned > 0).astype(np.uint8)
    neighbor_count_thinned = cv2.filter2D(binary_thinned, -1, k) - binary_thinned
    thick_pixels_thinned = np.count_nonzero((binary_thinned == 1) & (neighbor_count_thinned > 2))
    print(f"After thinning: {np.count_nonzero(skel_thinned > 0)} pixels, {thick_pixels_thinned} thick pixels")
    
    cv2.imwrite(f"test_skel_{method}_thinned.png", skel_thinned)

print("\nDone.")
