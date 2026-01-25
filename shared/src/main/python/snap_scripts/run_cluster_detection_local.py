import os
import sys

import ClusterDetection
import cv2
import numpy as np
from numpy.typing import NDArray

# Add the directory containing ClusterDetection.py to the Python path
# This assumes run_cluster_detection_local.py is in the same directory as ClusterDetection.py
script_dir = os.path.dirname(__file__)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)


def main():
    # --- Configuration ---
    # Path to a sample image. Make sure this image exists!
    # You can place a sample image (e.g., 'sample_image.jpg') in the same directory
    # as this script, or provide an absolute path.
    sample_image_path = os.path.join(script_dir, "sample_image.jpg")

    # Dummy llrobot data (as expected by runPipeline)
    # This list typically contains values like robot pose, camera intrinsics, etc.
    # For local testing, these values don't significantly impact the vision processing itself,
    # but are part of the expected signature.
    llrobot_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # --- Load Image ---
    if not os.path.exists(sample_image_path):
        print(f"ERROR: Sample image not found at '{sample_image_path}'")
        print(
            "Please place a sample image (e.g., 'sample_image.jpg') in the same directory as this script,"
        )
        print("or update 'sample_image_path' to point to a valid image file.")
        return

    loaded_image = cv2.imread(sample_image_path)

    if loaded_image is None:
        print(
            f"ERROR: Could not load image from '{sample_image_path}'. Check file path and integrity."
        )
        return

    # Ensure the image has the correct type for the pipeline
    image: NDArray[np.uint8] = loaded_image.astype(np.uint8)

    print(f"Successfully loaded image: {sample_image_path} with shape {image.shape}")

    # Keep a copy of the original for side-by-side comparison
    # (the pipeline draws on the image in place, same as on Limelight)
    original_image = image.copy()

    # --- Run the Pipeline ---
    print("Running ClusterDetection pipeline...")
    # Return order per Limelight API: (contour, image, llpython)
    best_contour, processed_image, llpython_output = ClusterDetection.runPipeline(
        image, llrobot_data
    )
    print("Pipeline finished.")

    # --- Display Results ---
    print("\n--- Pipeline Output ---")
    print(f"llpython data: {llpython_output}")
    print(f"  [0] cx: {llpython_output[0]}")
    print(f"  [1] cy: {llpython_output[1]}")
    print(f"  [2] area: {llpython_output[2]}")
    print(f"  [3] cluster_count: {llpython_output[3]}")
    print(f"  [4] score: {llpython_output[4]}")
    print(f"Best contour size: {len(best_contour)} points")

    # The processed_image now includes all visualizations from ClusterDetection.py
    # (contours, crosshair, text overlays) so we can display it directly
    cv2.imshow("Original Image", original_image)
    cv2.imshow("Processed Image (Limelight View)", processed_image)
    cv2.waitKey(0)  # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
