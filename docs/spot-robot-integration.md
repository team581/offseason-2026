# Vision → robot integration

Spot turns camera images into possible camera positions. The robot checks those results, converts them into robot positions, and uses them to correct its driving estimate. This guide follows startup, then one observation through the system.

**Current state:** `SpotCameraProfiles.CAMERAS` is empty and `FUSION_ENABLED` is `false`. The planned cameras are `left`, `right`, `front`, and `back`, all fixed to the chassis. Measured camera settings and hardware qualification are still needed before vision can correct the robot's pose. Heading correction is also disabled.

```mermaid
flowchart LR
  Config[Robot configuration] --> Spot[Spot capture and solve]
  Spot --> IO[SpotVisionIO]
  IO --> Check[VisionProcessor]
  History[CaptureHistory] --> Check
  Check --> Queue[MeasurementQueue]
  Queue --> Localization[Localization]
  Localization --> Estimator[Drive pose estimate]
  Localization --> Trust[TrustFactor]
```

Robot source links below point into `offseason-bot/src/main/java/frc/robot/`.

## 1. Define what Spot should run

[SpotCameraProfiles.java](../offseason-bot/src/main/java/frc/robot/vision/SpotCameraProfiles.java) owns the robot's camera definitions, measured mounts, field layout, and fusion settings. Keeping these together makes the robot's physical setup explicit.

[SpotProtocol.java](../offseason-bot/src/main/java/frc/robot/vision/SpotProtocol.java) builds the messages, converts pose formats, and hashes the configuration. The hash lets both computers confirm they are using the same settings.

[generated/SpotProtos.java](../offseason-bot/src/main/java/frc/robot/vision/generated/SpotProtos.java) provides the generated message classes used to encode and decode packets. Its source is `proto/vision_v2.proto` in the Spot repository; regenerate it when the contract changes.

## 2. Connect vision to the robot

[Robot.java](../offseason-bot/src/main/java/frc/robot/Robot.java) constructs `SpotVisionIO` and `Vision`, supplying the configuration, a fresh session ID, the robot clock, and drivetrain/IMU history. That gives vision the context needed to interpret delayed camera results.

[config/DSOptions.java](../offseason-bot/src/main/java/frc/robot/config/DSOptions.java) supplies `UseVision` and pit-mode controls. `Robot.java` uses them to allow or suppress pose correction while observation checks and diagnostics continue.

## 3. Start each loop with capture-time context

[Vision.java](../offseason-bot/src/main/java/frc/robot/vision/Vision.java) coordinates the vision subsystem. Its `collectInputs()` first records the current motion and tilt, then reads Spot inputs once for the loop.

[CaptureHistory.java](../offseason-bot/src/main/java/frc/robot/vision/CaptureHistory.java) retrieves pose, speed, and tilt at the image's capture time. The robot may have moved since then, so comparing against its current position would give misleading results.

[util/scheduling/SubsystemPriority.java](../offseason-bot/src/main/java/frc/robot/util/scheduling/SubsystemPriority.java) places input collection in IMU → Vision → Localization order, so localization receives this loop's prepared results.

## 4. Send the request and let Spot solve

[SpotVisionIO.java](../offseason-bot/src/main/java/frc/robot/vision/SpotVisionIO.java) publishes the configuration and robot state, including the requested tag IDs, under `/Spot/v2` using NetworkTables.

On the Jetson, these files in `spot-2026/src/spot_vision/` produce the response:

- `beta_transport.py` receives the request and later publishes acknowledgements, health, and observations. It provides the connection and synchronized timing needed for usable results.
- `beta_runtime.py` applies the configuration using saved camera bindings and calibration, then coordinates capture and processing.
- `beta_capture.py` reads images with their capture timing, preserving when each observation happened.
- `beta_processing.py` detects tags and filters them to the robot's requested IDs before solving.
- `pose.py` solves the camera's field pose, supplying candidates for the robot to evaluate.

## 5. Receive one consistent input snapshot

`SpotVisionIO.java` checks configuration acknowledgements, service health, packet identity, and timestamps before returning observations. It uses the NetworkTables capture timestamp in FPGA seconds so later checks share the robot's clock.

[VisionIO.java](../offseason-bot/src/main/java/frc/robot/vision/VisionIO.java) defines that snapshot and the commands/feedback exposed by the connection. This boundary also lets tests supply inputs without a physical Jetson.

## 6. Decide whether an observation is usable

[VisionProcessor.java](../offseason-bot/src/main/java/frc/robot/vision/VisionProcessor.java) checks configuration, calibration, age, duplicates, tag IDs, ambiguity, field bounds, tilt, and agreement with capture-time history. These checks prevent a plausible-looking but unsuitable result from shifting the robot's estimate.

It converts camera pose into robot pose using the inverse measured camera mount, then assigns uncertainty. The output is [results/VisionMeasurement.java](../offseason-bot/src/main/java/frc/robot/vision/results/VisionMeasurement.java): an immutable pose, capture time, source, tag list, and uncertainty that can safely pass between stages.

## 7. Put accepted results in time order

[MeasurementQueue.java](../offseason-bot/src/main/java/frc/robot/vision/MeasurementQueue.java) holds results briefly and releases them oldest first across cameras. Network arrival order can differ from capture order; the default reorder window is 20 ms.

`Vision.java` queues results only when fusion is allowed. Before handing them off, it increases uncertainty conservatively to avoid overcounting overlapping camera evidence.

## 8. Correct the driving estimate, then update trust

[localization/Localization.java](../offseason-bot/src/main/java/frc/robot/localization/Localization.java) drains the measurements, rechecks eligibility and pose history, converts each capture timestamp to CTRE time, and calls `drivetrain.addVisionMeasurement(...)`. It is the single owner of inserting vision into the drive estimator.

Only after submission does it notify `Vision.measurementFused(...)` and update [TrustFactor.java](../offseason-bot/src/main/java/frc/robot/vision/TrustFactor.java). Trust reflects submitted vision evidence and degrades with travel, elapsed time, and collisions. This keeps an accepted observation from improving trust before it actually affects localization.

Pose or gyro resets also clear pending vision and history, preventing older frames from undoing the reset.

## 9. Use the result and choose the next tags

[health/HealthManager.java](../offseason-bot/src/main/java/frc/robot/health/HealthManager.java) reads cached camera availability. Camera health and pose trust answer different questions: a working camera can see no tags while localization trust ages.

[robot_manager/RobotManager.java](../offseason-bot/src/main/java/frc/robot/robot_manager/RobotManager.java) uses localization pose/trust for robot decisions and requests broad or alliance-hub tags for subsequent observations.

[VisionState.java](../offseason-bot/src/main/java/frc/robot/vision/VisionState.java) names the three selection states: `TAGS`, `WAITING_FOR_HUB_TAGS`, and `HUB_TAGS`. `Vision.java` acquires broadly, narrows after fresh accepted alliance-hub evidence, and returns to broad acquisition after 0.5 seconds without it. Selection changes discard pending results from the previous policy.

Finally, `Vision.java` logs status and asks `SpotVisionIO.java` to publish decision feedback, including whether a result was actually fused. That makes the full path from observation to pose correction visible during debugging.
