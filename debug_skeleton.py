"""Debug script to test skeleton path tracing with simple shapes."""
import cv2
import numpy as np
import sys

# Create a simple test image with a thick line
img = np.zeros((200, 200), dtype=np.uint8)
cv2.line(img, (20, 100), (180, 100), 255, 8)  # thick horizontal line
cv2.line(img, (100, 20), (100, 180), 255, 8)  # thick vertical line

# Save test image
cv2.imwrite("test_input.png", img)

# Threshold
mask = img.copy()

# Import the functions from v16
sys.path.insert(0, '.')
from v16 import extract_centerline, _prune_skeleton, _trace_skeleton_paths

# Test each method
for method in ['zhang_suen', 'distance']:
    print(f"\n=== Testing method: {method} ===")
    skel = extract_centerline(mask, method=method)
    cv2.imwrite(f"test_skel_{method}.png", skel)
    
    # Count skeleton pixels
    skel_pixels = np.count_nonzero(skel > 0)
    print(f"Skeleton pixels: {skel_pixels}")
    
    # Prune
    skel_pruned = _prune_skeleton(skel, min_len=3)
    cv2.imwrite(f"test_skel_{method}_pruned.png", skel_pruned)
    skel_pruned_pixels = np.count_nonzero(skel_pruned > 0)
    print(f"After pruning: {skel_pruned_pixels} pixels")
    
    # Trace
    strokes = _trace_skeleton_paths(skel_pruned, min_px=5)
    print(f"Strokes found: {len(strokes)}")
    for i, s in enumerate(strokes):
        print(f"  Stroke {i}: {len(s)} points")
    
    # Also try with lower min_px
    strokes2 = _trace_skeleton_paths(skel_pruned, min_px=1)
    print(f"Strokes found (min_px=1): {len(strokes2)}")
    for i, s in enumerate(strokes2):
        print(f"  Stroke {i}: {len(s)} points")

print("\nDone. Check test_skel_*.png files.")
