# FTC Vision-Based Autonomous and Teleop Code - README

## Introduction

This code provides a robust example of using vision processing in FTC (FIRST Tech Challenge) for both autonomous and teleoperated robot control. It's designed to detect colored cubes (red, blue, yellow) using a webcam and OpenCV, and then perform actions like approaching and grabbing the detected cube autonomously. The code also includes a teleop mode with dual gamepad control.

This README focuses on the **innovative aspects** of the code, particularly the vision processing pipeline and calculation methods, to help other FTC teams understand and adapt these techniques for their own robots.

## Core Innovations

This code demonstrates several innovative approaches for FTC robotics, centered around real-time vision processing to enable sophisticated autonomous actions:

1. **Vision-Guided Autonomous Actions:** The robot uses continuous vision processing to detect and react to its environment in real-time, enabling dynamic autonomous behaviors like approaching and grabbing objects without pre-programmed paths. This is a significant step beyond purely pre-programmed autonomous routines, allowing for greater adaptability on the field.

2. **Real-Time Image Processing Pipeline:** The code implements an efficient OpenCV pipeline optimized for FTC's computational constraints. It achieves robust color detection and object localization within each frame, enabling timely reactions. The pipeline is designed for speed and accuracy within the limited processing power available on FTC control systems.

3. **Integrated Motion Planning with Vision Feedback:** The vision system is tightly coupled with Road Runner motion planning. Vision-derived movement commands are seamlessly converted into smooth, accurate robot motions using Road Runner's trajectory generation, combining the responsiveness of vision with the precision of motion profiling. This integration allows for complex, vision-guided maneuvers that are both smooth and precise.

## Innovative Calculation Methods and Image Processing Pipeline

Let's delve into the key innovative parts of the code, focusing on the image processing pipeline and the calculation methods used to translate vision data into robot actions.

### 1. Efficient Image Processing Pipeline (`ColorDetectionPipelineImpl`)

This pipeline is designed for speed and robustness in FTC environments. Here are the key steps and innovations:

**a) HSV Color Space for Robust Color Segmentation:**

```
Imgproc.cvtColor(outputImage, hsvImage, Imgproc.COLOR_RGB2HSV);
```

Innovation: Instead of using RGB color space, the code converts the image to HSV (Hue, Saturation, Value). HSV is more robust to lighting changes for color detection because:

- **Hue:** Represents the pure color (like red, blue, yellow) and is less affected by brightness variations.
- **Saturation:** Represents the intensity or purity of the color.
- **Value (Brightness):** Represents the lightness or darkness of the color.

By defining color ranges in HSV, the code can reliably detect colors even when lighting conditions change during a match, which is a significant advantage over simple RGB thresholding. This is crucial for competition environments where lighting is not always consistent.

**b) Multi-Masking for Robust Color Range Detection:**

```
Core.inRange(hsvImage, RED_LOWER_1, RED_UPPER_1, redMask1);
Core.inRange(hsvImage, RED_LOWER_2, RED_UPPER_2, redMask2);
Core.bitwise_or(redMask1, redMask2, mask);

Core.inRange(hsvImage, BLUE_LOWER, BLUE_UPPER, blueMask);
Core.bitwise_or(mask, blueMask, mask);

Core.inRange(hsvImage, YELLOW_LOWER, YELLOW_UPPER, yellowMask);
Core.bitwise_or(mask, yellowMask, mask);
```

Innovation: For colors like red that "wrap around" in the Hue circle (0° and 360° are both red), the code uses multiple masks (redMask1, redMask2) to capture the full range of red hues. A bitwise OR operation (`Core.bitwise_or`) combines these masks into a single mask for red. This ensures that red objects are detected even if their hue values fall at the edges of a single HSV range. The same principle is applied to combine all color masks (red, blue, yellow) into a final mask. This technique enhances red color detection accuracy and robustness.

**c) Contour Detection and Filtering for Object Isolation:**

```
Imgproc.findContours(localMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

for (MatOfPoint contour : contours) {
    if (Imgproc.contourArea(contour) < 500) continue; // Area filtering

    // ... size and dimension filtering ...
}
```

Innovation: After masking, `Imgproc.findContours` efficiently identifies potential object regions (contours) in the binary mask. The code then implements a series of filtering steps to eliminate noise and non-cube objects:

- **Area Filtering:** `Imgproc.contourArea(contour) < 500` - Removes very small contours that are likely noise, significantly reducing false positives from minor image artifacts.
- **Size and Dimension Filtering:** Checks `boundingBox.width` and `boundingBox.height` against `MIN_REGION_WIDTH`, `MIN_REGION_HEIGHT`, `SINGLE_SIDE_MAX_SIZE`, and `DOUBLE_SIDE_MAX_SIZE` to filter out regions that are too small, too large, or have unrealistic dimensions for cubes. This sophisticated filtering based on multiple criteria dramatically improves the reliability of cube detection.

These filtering steps are crucial for making the vision system robust and preventing false detections, especially in the noisy environment of an FTC competition.

**d) Real-Time Processing Focus:**

The pipeline prioritizes speed. Operations are chosen to be computationally efficient for real-time performance on FTC robots. Releasing Mat objects (`mask.release()`, `hsvImage.release()`, etc.) after use is critical for memory management and preventing lag in continuous processing. Efficient memory management is key to maintaining a high frame rate and responsive vision system on resource-constrained FTC robots.

### 2. Innovative Calculation Methods for Robot Control

The code employs several key calculations to translate vision data into robot actions:

**a) Pixel to Centimeter Conversion (PIXELS_TO_CM_RATIO):**

```
private static final double PIXELS_TO_CM_RATIO = 0.021875;
clawCenterXPixel = (int) (CLAW_CENTER_X_CM / PIXELS_TO_CM_RATIO + (double) width / 2);
clawCenterXCm = calculateWorldCoordinatesX(clawCenterXPixel, height);
```

Innovation: The `PIXELS_TO_CM_RATIO` is a crucial calibration constant that allows the vision system to understand distances in real-world units (centimeters) instead of just pixels. This is essential for accurate robot movements. This ratio bridges the gap between the image space and the physical robot's workspace.

The code uses this ratio to:

- Calculate the pixel position of the claw center based on its real-world centimeter coordinates (`CLAW_CENTER_X_CM`, `CLAW_CENTER_Y_CM`). This allows for defining robot-centric coordinates in real-world units and mapping them back to the image.
- Convert pixel coordinates of detected cubes back to world coordinates (`calculateWorldCoordinatesX`, `calculateWorldCoordinatesY`). This is the core of enabling vision-guided motion in real-world units.

**b) World Coordinate Calculation (`calculateWorldCoordinatesX`, `calculateWorldCoordinatesY`):**

```
private double calculateWorldCoordinatesX(int centerXPixel, int imageHeight) {
    return (centerXPixel - imageHeight / 2.0) * PIXELS_TO_CM_RATIO;
}

private double calculateWorldCoordinatesY(int centerYPixel, int imageHeight) {
    int pixelDistanceFromBody = imageHeight - centerYPixel;
    return (pixelDistanceFromBody * PIXELS_TO_CM_RATIO) - CLAW_OFFSET_FROM_CAMERA_CM;
}
```

Innovation: These methods convert 2D pixel coordinates from the camera image into a 2D world coordinate system relative to the robot. This transformation is fundamental for relating what the camera sees to where the robot needs to move.

- `calculateWorldCoordinatesX`: Assumes the camera is roughly centered horizontally and calculates the X-coordinate based on the horizontal pixel offset from the image center and the `PIXELS_TO_CM_RATIO`. This provides a horizontal position relative to the robot's center.
- `calculateWorldCoordinatesY`: Calculates the Y-coordinate (distance from the robot) based on the vertical pixel position, `PIXELS_TO_CM_RATIO`, and `CLAW_OFFSET_FROM_CAMERA_CM` to account for the camera's mounting position relative to the claw. The `CLAW_OFFSET_FROM_CAMERA_CM` is a critical calibration parameter for accurate depth estimation.

**c) Movement Command Calculation (`calculateMovementAndServoOffset`):**

```
moveForward = closestCube.centerYCm - clawCenterYCm;
moveSideways = closestCube.centerXCm - clawCenterXCm - CLAW_HORIZONTAL_ERROR_CM;
```

Innovation: The code calculates `moveForward` and `moveSideways` commands directly in centimeters based on the difference between the detected cube's world coordinates (`closestCube.centerYCm`, `closestCube.centerXCm`) and the claw's world coordinates (`clawCenterYCm`, `clawCenterXCm`). This direct calculation simplifies the control logic.

This direct centimeter-based calculation makes the movement commands intuitive and directly related to the physical displacement needed to reach the target. `CLAW_HORIZONTAL_ERROR_CM` is an experimental correction factor to refine sideways movements, demonstrating a practical approach to improving real-world accuracy through empirical tuning.

**d) Servo Offset Calculation for Claw Alignment:**

```
double clawAngleDegrees = closestCube.angleDegrees;
double angleDeviation = clawAngleDegrees + 90;
double servoValueChange = angleDeviation / 180.0;
double servoValue = 0.54 + servoValueChange;
servoValue = WebcamExample.wrapAroundServoValue(servoValue);
servoPositionOffset = servoValue - SERVO_CENTER_POSITION_HENG

;
```

Innovation: To attempt to align the claw with the cube's orientation, the code calculates a `servoPositionOffset` for the horizontal claw servo (`clawHengServo`) based on the detected `closestCube.angleDegrees`. This is a rudimentary but effective attempt at orientation-aware manipulation.

It maps the cube's angle to a servo value change and adjusts the servo position relative to its center position (`SERVO_CENTER_POSITION_HENG`). `wrapAroundServoValue` ensures the servo value stays within the valid 0-1 range. This demonstrates a basic approach to integrating object orientation into robot actions.

**e) Moving Average Filtering for Smooth Motion:**

```
moveForward = applyMovingAverage(moveForwardHistory, moveForward);
moveSideways = applyMovingAverage(moveSidewaysHistory, moveSideways);
```

Innovation: A moving average filter is applied to the `moveForward` and `moveSideways` commands. This is a simple but effective technique to smooth out the commands and reduce jitter caused by noise in the vision processing. It improves the stability and smoothness of the robot's movements, especially in continuous vision-guided control. This filtering is essential for achieving reliable and predictable robot behavior in vision-based autonomous routines.

### 3. Integration with Road Runner for Motion Planning

```
Action approachMovement = drive.actionBuilder(drive.pose)
        .splineToConstantHeading(targetPosition, 0)
        .build();
Actions.runBlocking(approachMovement);
```

Innovation: The code leverages the Road Runner library to execute the vision-derived movement commands. Instead of directly setting motor powers, it creates Road Runner Actions using `drive.actionBuilder()`. This is a key innovation as it combines real-time vision with advanced motion control.

`splineToConstantHeading()` is used to generate smooth, spline-based trajectories towards the `targetPosition`. Road Runner handles the low-level motor control and motion profiling to achieve accurate and controlled movements, even when guided by real-time vision input. `Actions.runBlocking()` ensures that the robot completes the motion before proceeding to the next state, creating a robust and predictable autonomous sequence. This integration allows for complex, vision-guided maneuvers that are both smooth and precise, a significant advantage in competitive FTC scenarios.

## Adapting and Extending for Your Team

When adapting this code, focus on:

- **Calibration:** Accurately calibrate `PIXELS_TO_CM_RATIO`, `CLAW_OFFSET_FROM_CAMERA_CM`, `CLAW_CENTER_X_CM`, `CLAW_CENTER_Y_CM`, `CLAW_HORIZONTAL_ERROR_CM`, and meticulously tune the HSV color thresholds for your specific lighting conditions and cube colors. Calibration is the absolute foundation for accurate vision-guided control; without it, the system will not function reliably.

- **Pipeline Tuning:** Experiment with different image processing parameters (e.g., filter sizes, threshold values, contour filtering criteria) within `ColorDetectionPipelineImpl` to optimize performance for your specific camera, lighting, and game elements. Fine-tuning the pipeline is crucial for maximizing detection accuracy and minimizing false positives in your competition environment.

- **Autonomous Strategy:** Extend the AutoState machine and the `processAutoStateMachine()` method to implement more complex autonomous routines beyond just approaching and grabbing. Use the vision pipeline to guide navigation, object manipulation, and scoring actions. Consider adding states for navigation to specific field locations, manipulating different game elements, or reacting to opponent robots.

- **Servo Control Refinement:** Adapt the servo control sequences in `processFrameAndGrab()` and the servo offset calculation to match your robot's specific claw and arm mechanisms. Consider more advanced servo control strategies if needed, such as incorporating feedback from sensors to ensure a successful grab, or developing more sophisticated orientation-aware manipulation techniques.

## Conclusion

This code provides a strong starting point and a valuable learning resource for FTC teams looking to implement advanced vision-based autonomous capabilities. By focusing on the innovative aspects of the image processing pipeline and calculation methods, and by diligently calibrating and tuning the system, your team can create robust and competitive vision-guided robots. Remember to experiment, test rigorously, and iterate on the design to achieve success in your FTC season.
```

Let me know if you'd like any modifications!