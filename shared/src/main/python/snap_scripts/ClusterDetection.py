import cv2
import numpy as np


def runPipeline(image, llrobot):
    # 1. Initialize variables
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 2. Thresholding (Adjust these for the 2026 yellow game piece)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    # 3. Find all potential targets
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    all_targets = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 50:  # Filter out noise
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                all_targets.append((cx, cy, area))

    # 4. Grouping Logic: Find the "Best" Cluster
    # We want a target that has the most neighbors within a 100-pixel radius
    best_target = [0, 0, 0, 0, 0]  # tx, ty, ta, count, cluster_score
    max_score = -1

    proximity_threshold = 100  # Adjust based on camera height/distance

    for i, t1 in enumerate(all_targets):
        cluster_count = 0
        for j, t2 in enumerate(all_targets):
            # Calculate distance between game pieces
            dist = np.sqrt((t1[0] - t2[0]) ** 2 + (t1[1] - t2[1]) ** 2)
            if dist < proximity_threshold:
                cluster_count += 1

        # Score = number of items in cluster * area of the primary item
        score = cluster_count * t1[2]

        if score > max_score:
            max_score = score
            # Convert to Limelight crosshair space (-1 to 1 or -30 to 30 deg)
            # This example returns raw center for simpler visualization
            best_target = [t1[0], t1[1], t1[2], cluster_count, score]

    # 5. Output to NetworkTables (llpython array)
    # [tx, ty, area, num_in_cluster, score]
    return contours, best_target, image
