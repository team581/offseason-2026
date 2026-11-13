# Configure Spot cameras

## Quick hardware reference

- **SWYFT USB AprilTag Camera:** 1280 × 800 = **1,024,000 pixels (1.024 MP)**; monochrome global shutter; USB 2.0; **MJPEG only**. Advertised MJPEG maxima: 210 FPS at 160×120, 320×240, 352×288, 640×400; 120 FPS at 640×480, 800×600, 1024×768, 1280×720, 1280×800. See the [SWYFT specifications](https://shop.swyftrobotics.com/products/swyft-usb-camera).
- **Arducam UC-844 Rev.B / B0332:** OV9281 monochrome global shutter; 1280 × 800 = **1,024,000 pixels (1.024 MP)**; 3 µm sensor pixels; USB 2.0. Its [manufacturer datasheet](https://uctronics.com/download/Amazon/B0332_OV9281_Global_Shutter_UVC_Camera_Datasheet.pdf) lists MJPEG at 100 FPS for 1280×800, 800×600, 640×480, 320×240; YUY2 at 10 FPS for 1280×800 and 1280×720. Older documentation lists different MJPEG rates: inspect your firmware's actual descriptors.
- **Recorded team results:** the camera called Swift/Generic USB Camera in older notes used MJPEG 640×480 at requested 120 FPS; higher advertised resolutions timed out. Attached Arducams were recorded as UC762, with MJPEG 1280×720 at requested 120 FPS. Those identities/rates are historical records, not proof of UC-844 Rev.B capabilities. Confirm the board label and enumerated device before copying settings.

Advertised/requested FPS is not measured delivered FPS or detection FPS. Test all cameras simultaneously on their actual USB topology. YUYV is uncompressed USB traffic; MJPEG is compressed USB traffic. Spot can convert decoded MJPEG into YUYV in Jetson memory without sending YUYV over USB. A monochrome sensor can still expose color controls that have little or no image effect.

This reference was checked on October 18, 2026. The Jetson was unreachable during this documentation update, so current per-device firmware ranges and menu subsets have not been re-enumerated. Product specifications, historical observations, and generic control meanings below are deliberately distinguished.

## Robot defaults and Spot changes

Edit `offseason-bot/src/main/deploy/spot-config.json` for everything Spot reads
about cameras and processing. Its `$schema` gives VS Code completion and checks;
exposure values are in microseconds, and mounts are metres plus roll/pitch/yaw
degrees in WPILib axes. [VisionConfig.java](../offseason-bot/src/main/java/frc/robot/vision/VisionConfig.java)
holds what robot code also uses (field layout, tag size, observation age limits) and
robot-only policy such as fusion. See Spot's
[robot configuration guide](https://spot.frc581.com/robot-config/) for which
settings belong where.

Dashboard edits take priority over the deployed file across reconnects, restarts,
and deployments. Spot's **Changes** page lists them; copy `spot-config.json` from
there into the robot project and deploy, and each change clears once the deployed
file matches it.

See Spot's [camera setup guide](https://spot.frc581.com/operator/cameras/) for
binding, tuning, calibration, and export steps. Set every control the camera
depends on explicitly; omitted controls have no robot-owned default.

## Exposure and automatic-control values

The standard V4L2 exposure-mode menu has the meanings below. A camera may expose only some entries; select only values printed by its own menu. See the [Linux camera-control reference](https://docs.kernel.org/userspace-api/media/v4l/ext-ctrls-camera.html).

- **`auto_exposure` or `exposure_auto`:** `0` = automatic exposure and iris; `1` = manual exposure and iris; `2` = manual exposure with automatic iris; `3` = automatic exposure with manual iris. Fixed-lens USB cameras commonly expose only `1` (manual) and `3` (automatic). Do not use OpenCV's unrelated fractional auto-exposure values such as 0.25/0.75 here.
- **`exposure_time_absolute` or `exposure_absolute`:** integration duration. In `spot-config.json` and Spot use ms: `1` = 1 ms; `5` = 5 ms. Raw V4L2 uses 100 µs ticks, so those become native `10` and `50`. Set manual exposure mode first; Spot orders automatic-control writes ahead of dependent manual writes.
- **`autogain`:** `0` = manual gain; `1` = automatic gain. If supported, disable it before pinning `gain`.
- **`white_balance_automatic` or `white_balance_temperature_auto`:** `0` = manual white balance; `1` = automatic. Disable it before writing a temperature. A monochrome camera may advertise these controls without useful color effects.
- **`power_line_frequency`:** `0` = disabled; `1` = 50 Hz; `2` = 60 Hz; `3` = automatic. Only advertised entries are legal. This is an anti-flicker lighting setting, not capture FPS. See the [Linux user-control reference](https://docs.kernel.org/userspace-api/media/v4l/control.html).

Exposure must fit the capture interval and the device's own range/step. At 100 µs steps the interval limits are 33,300 µs at 30 FPS, 16,600 µs at 60 FPS, and 8,300 µs at 120 FPS. Shorter exposure generally reduces motion blur and collects less light; longer exposure collects more light but can smear tags. Historical team readings found a 5,000 µs minimum for the SWYFT/Generic camera and used 1,000 µs on the attached Arducams. These are firmware observations, not universal minimums for every camera sold under those names.

## Every image control Spot labels explicitly

The following list covers every name/alias in Spot's current image-control descriptions. It explains meaning; it does not claim every control exists on either camera. For each integer, use the actual `min`, `max`, `step` and menu, not an assumed 0–100 scale. Standard semantics are described by the [Linux user-control reference](https://docs.kernel.org/userspace-api/media/v4l/control.html); units and availability come from the device.

- **`auto_exposure`, `exposure_auto`:** exposure mode menu; values above.
- **`exposure_time_absolute`, `exposure_absolute`:** exposure duration in ms in robot code; values above.
- **`autogain`:** automatic/manual gain switch; values above.
- **`gain`:** image amplification. Higher generally brightens the image and increases noise. It is not exposure duration and has no universal dB conversion or range.
- **`brightness`:** black-level/luminance offset. Raising it can lift dark pixels without collecting more light. Zero is not necessarily the default or midpoint.
- **`black_level`:** a separately named black-level control on devices exposing it. It is not an interchangeable spelling to invent when only brightness is advertised.
- **`contrast`:** separation of bright and dark intensities. Excess can clip tag edges/highlights; device scale is arbitrary.
- **`gamma`:** midtone response. The number may use a firmware-specific scale; do not assume the value 1 means linear gamma.
- **`sharpness`:** edge enhancement. The advertised minimum disables the standard sharpness filter; larger values strengthen it and can create halos. Use the actual minimum rather than assuming zero is allowed.
- **`backlight_compensation`:** processing to brighten subjects against a bright background. The standard minimum disables it; larger values increase the device's compensation. Not a substitute for exposure tuning.
- **`power_line_frequency`:** anti-flicker menu; values above.
- **`white_balance_automatic`, `white_balance_temperature_auto`:** automatic/manual white balance switch; values above.
- **`white_balance_temperature`:** manual color temperature in kelvin, within device limits. Lower values represent warmer illumination; higher values represent cooler illumination. Automatic white balance must be off. May have no meaningful effect on a monochrome image.
- **`saturation`:** color intensity. The standard zero setting removes chroma when zero is available; higher values increase color strength. Usually unhelpful for a monochrome AprilTag camera.
- **`hue`:** color-balance adjustment on the advertised scale. It does not rotate the image or change camera heading; usually unhelpful on monochrome sensors.

## Other controls, limits and unsupported options

`spot-config.json` accepts other valid control names, and Spot discovery reads advertised scalar `int`, `bool`, `menu`, and `intmenu` controls. There is no fixed universal list across all UVC cameras. A boolean normally uses 0/1; a menu uses the exact listed integer, which can have gaps. Device flags such as inactive, read-only, disabled and grabbed can prevent changes. Button, string, array and write-only action controls are not supported by this policy path.

Examples you might see on other devices include `exposure_dynamic_framerate`/`exposure_auto_priority` (0 keeps auto exposure from varying FPS; 1 permits it), `focus_automatic_continuous`/`focus_auto` (0 manual, 1 autofocus), `focus_absolute` (driver-specific position), `zoom_absolute` (driver-specific zoom), and pan/tilt/iris controls. Treat those spellings as examples: use your device's actual enumeration. Do not infer unsupported motorized focus, zoom or iris from the presence of an M12 lens. SWYFT's manufacturer lists focus/zoom/pan/tilt/iris as unsupported. Focus/zoom/crop/flip changes can invalidate calibration; change them only with a matching optical configuration and fresh validation.

`convert_rgb`/`CAP_PROP_CONVERT_RGB` is a host OpenCV property, not a portable UVC camera policy. It is not an extra `pixel_format` or a control to add unless the device actually advertises an applicable scalar control. Detector backend, decimation, camera preview encoding and detector FPS are also not `Control` entries.

## Get the exact options for your physical camera

The complete current per-device list is in the camera's Spot Config/Advanced panel: names, labels, menus, units, min/max/step, requested values, actual readback and policy locks. Exposure values there are already in µs. Use the discovered list to decide what is supported; manufacturer lists and this reference explain it but cannot supply universal firmware ranges.

For raw capability inspection on the Jetson, these commands only read descriptors/controls and do not start another capture stream. Select the image-capture node, not a metadata node. `/dev/video0` is an example for inspection only; save physical bindings using stable identity/USB paths.

```sh
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video0 --list-ctrls-menus
```

A hypothetical line `min=50 max=10000 step=1 default=166 value=50` for `exposure_time_absolute` means native ticks: 5–1,000 ms, 0.1 ms increments, 16.6 ms device default, 5 ms current. The capture interval can impose a lower usable maximum. For non-exposure integer controls use the listed units directly. Legal values satisfy min ≤ value ≤ max and `(value - min) % step == 0`; menu values must also appear in the menu. Raw `default` is the device default, not necessarily the robot-code default or saved GUI override.

## Commissioning

Verify measured geometry, supported capture modes and actual readback, calibration,
capture timing, multi-camera load, and reconnect/reset behavior on the robot before
relying on vision fusion. Software compilation does not establish hardware accuracy.
