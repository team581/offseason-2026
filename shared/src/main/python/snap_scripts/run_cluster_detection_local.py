import cv2
import numpy as np
import os
import sys

# Add the directory containing ClusterDetection.py to the Python path
# This assumes run_cluster_detection_local.py is in the same directory as ClusterDetection.py
script_dir = os.path.dirname(__file__)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import ClusterDetection

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
        print("Please place a sample image (e.g., 'sample_image.jpg') in the same directory as this script,")
        print("or update 'sample_image_path' to point to a valid image file.")
        return

    image = cv2.imread(sample_image_path)

    if image is None:
        print(f"ERROR: Could not load image from '{sample_image_path}'. Check file path and integrity.")
        return

    print(f"Successfully loaded image: {sample_image_path} with shape {image.shape}")

    # --- Run the Pipeline ---
    print("Running ClusterDetection pipeline...")
    contours, llpython_output, processed_image = ClusterDetection.runPipeline(image, llrobot_data)
    print("Pipeline finished.")

    # --- Display Results ---
    print("\n--- Pipeline Output ---")
    print(f"llpython data: {llpython_output}")
    print(f"Number of contours found: {len(contours)}")

    # Make a copy of the original image to draw on for visualization.
    # The 'processed_image' returned by runPipeline is currently the input image itself,
    # as the pipeline doesn't modify it directly.
    display_image = image.copy()

    # Draw all found contours in green
    if contours:
        cv2.drawContours(display_image, contours, -1, (0, 255, 0), 2) # Green contours, thickness 2

    # Draw the "best target" if one was identified (llpython_output will not be all zeros)
    # llpython_output is [tx, ty, area, num_in_cluster, score]
    # Check if tx and ty are non-zero, indicating a target was found
    if len(llpython_output) >= 2 and (llpython_output[0] != 0 or llpython_output[1] != 0):
        center_x = int(llpython_output[0])
        center_y = int(llpython_output[1])
        # Draw a red circle at the center of the best target
        cv2.circle(display_image, (center_x, center_y), 10, (0, 0, 255), -1) # Red circle, filled
        # Draw a crosshair for more precision
        cv2.line(display_image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(display_image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
        cv2.putText(display_image, f"Score: {llpython_output[4]:.0f}", (center_x + 15, center_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.imshow("Original Image", image)
    cv2.imshow("Processed Image with Detections", display_image)
    cv2.waitKey(0) # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
