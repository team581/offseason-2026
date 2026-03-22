import cv2
import numpy as np
from numpy.typing import NDArray


def runPipeline(
    image: NDArray[np.uint8], llrobot: list[float]
) -> tuple[np.ndarray, NDArray[np.uint8], list[int | float]]:
    """
    Main pipeline function called by Limelight.

    Args:
        image: The camera frame as a NumPy array in BGR color format.
        llrobot: Numeric array of values sent from the robot via NetworkTables.

    Returns:
        A tuple of (largestContour, image, llpython):
        - largestContour: The contour for Limelight's crosshair to latch onto
        - image: The (possibly modified) image array
        - llpython: Custom numeric data to send back via NetworkTables
    """
    # 1. Initialize variables
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 2. Thresholding (Adjust these for the 2026 yellow game piece)
    lower_yellow = np.array([49, 100, 50])
    upper_yellow = np.array([51, 73, 98])
    mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    # 3. Find all potential targets
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Track both contour and its computed properties
    all_targets: list[tuple[np.ndarray, int, int, float]] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 50:  # Filter out noise
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                all_targets.append((cnt, cx, cy, area))

    # 4. Grouping Logic: Find the "Best" Cluster
    # We want a target that has the most neighbors within a 100-pixel radius
    llpython: list[int | float] = [0, 0, 0, 0, 0]  # cx, cy, area, count, cluster_score
    best_contour: np.ndarray = np.array([[]])
    best_center: tuple[int, int] | None = None
    max_score: float = -1

    proximity_threshold = 100  # Adjust based on camera height/distance

    for cnt, cx1, cy1, area1 in all_targets:
        cluster_count = 0
        for _, cx2, cy2, _ in all_targets:
            # Calculate distance between game pieces
            dist = np.sqrt((cx1 - cx2) ** 2 + (cy1 - cy2) ** 2)
            if dist < proximity_threshold:
                cluster_count += 1

        # Score = number of items in cluster * area of the primary item
        score = cluster_count * area1

        if score > max_score:
            max_score = score
            best_contour = cnt
            best_center = (cx1, cy1)
            # Store custom data: [cx, cy, area, num_in_cluster, score]
            llpython = [cx1, cy1, area1, cluster_count, score]

    # 5. Draw visualizations on the image for debugging in Limelight web UI
    # Draw all detected contours in yellow (thin)
    if contours:
        cv2.drawContours(image, contours, -1, (0, 0, 255), 2)

    # Highlight the best contour in green (thick)
    # Note: Limelight draws its own crosshair on this contour automatically
    if len(best_contour) > 0 and len(best_contour[0]) > 0:
        cv2.drawContours(image, [best_contour], -1, (0, 255, 0), 2)

    # Display cluster info near best target (offset to avoid Limelight's crosshair)
    if best_center is not None:
        cx, cy = best_center
        cv2.putText(
            image,
            f"N:{int(llpython[3])} S:{int(llpython[4])}",
            (cx + 25, cy - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

    # Display total targets found (bottom-left to avoid Limelight's FPS/pipeline text)
    img_height = image.shape[0]
    cv2.putText(
        image,
        f"Targets: {len(all_targets)}",
        (10, img_height - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    # 6. Return in Limelight's expected order: contour, image, llpython
    # The contour is used by Limelight's crosshair (tx, ty, ta, etc.)
    return best_contour, image, llpython
