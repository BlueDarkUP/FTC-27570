# FTC Competition Teleop Code with Vision - README (Innovation Focus)
[We use a program for placing samples in high bars and have visual capabilities](https://github.com/BlueDarkUP/FTC-27570-INTO-THE-DEEP/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SingleStickWithArmWithVision.java)

## Introduction

This code represents a competition-ready FTC Teleop OpMode, incorporating advanced vision processing and sophisticated robot control. Built for team FTC-27570, this 1500-line codebase prioritizes robust functionality, driver control, and autonomous vision-assisted features.

This README highlights the **innovative aspects** of the code, focusing on the vision integration, advanced control mechanisms, and the overall design philosophy that makes this code suitable for competitive FTC scenarios.

## Core Innovations

This code goes beyond basic teleop, integrating several advanced features:

1. **Vision-Assisted Teleop:** Seamlessly integrates real-time vision processing into a driver-controlled OpMode. The vision system continuously detects cubes, providing valuable telemetry data to the drivers and enabling semi-autonomous features like vision-guided grabbing.

2. **Advanced Servo and Motor Control:** Implements precise servo control for complex mechanisms (claw, arm, slides) with features like:
    - **Debounced Button Inputs:** Prevents accidental multiple triggers of servo actions.
    - **Servo Speed Multiplier:** Allows for fine-grained manual servo control using joystick input.
    - **Pre-defined Servo Positions:** Uses constants for common servo states, improving code readability and maintainability.
    - **Motor Actions & Servo Actions:** Utilizes Road Runner Actions to encapsulate complex, sequential servo and motor movements, enabling more sophisticated automated routines within teleop.

3. **Integrated Odometry and IMU-Based Navigation:** Leverages a GoBilda Pinpoint odometry system and IMU for accurate pose estimation. Features include:
    - **IMU Resetting Routine:** A robust in-code IMU reset function, crucial for maintaining accurate heading during matches.
    - **Pose Recording:** Drivers can record robot poses (e.g., "Catch Pose", "Chamber Pose") on-the-fly during teleop, enabling quick execution of pre-defined autonomous sequences.
    - **Chamber Auto Drive Routine:** Demonstrates a complex autonomous sequence that utilizes recorded poses and vision for efficient cube collection (though commented out in the main loop, showcasing its capability).

4. **Multi-State Management:** Employs enums (`SlideState`, `AutoState`) to manage complex robot states, ensuring smooth transitions between different modes of operation (e.g., slide movement states, autonomous states).

5. **Comprehensive Telemetry:** Provides extensive telemetry data to the Driver Station, including:
    - **Robot Pose (Odometry & IMU):** Real-time position and heading information.
    - **Motor and Servo Status:** Encoder positions, target positions, power levels, servo positions.
    - **Vision Detection Data:** Detected cube properties (color, position, angle, size).
    - **Movement Commands:** Vision-derived movement instructions.
    - **System Status:** Auto state, slide state, recorded poses, drive speed multiplier, etc.

    This detailed telemetry is invaluable for driver awareness, debugging, and performance monitoring during matches.

## Key Code Innovations and Design

Let's examine the code's innovative sections:

### 1. Vision Integration in Teleop

While the vision pipeline (`ColorDetectionPipelineImpl`) is largely similar to the previous example (and equally innovative in its own right - see previous README), its **seamless integration into a Teleop OpMode** is a key innovation here.

- **Continuous Vision Processing:** The `ColorDetectionPipelineImpl` is continuously running in the background during Teleop, updating `detectedCubes` and `closestCube` in each frame.

- **Telemetry Display:** The `updateTelemetry()` method displays the real-time vision data, allowing drivers to see what the vision system is detecting and use this information to make informed driving decisions.

- **Semi-Autonomous Vision-Guided Actions (Example: `processFrameAndGrab`)**: Although the primary control is manual, the code retains the `processFrameAndGrab()` routine (and related `AutoState` machine). This demonstrates the *potential* for drivers to trigger semi-autonomous, vision-guided actions during teleop at the press of a button (though in this specific Teleop, the auto-grab is triggered by gamepad2.circle, it could be driver-initiated). This hybrid approach combines the flexibility of driver control with the precision of vision-based automation.

### 2. Advanced Control Mechanisms

**a) Debounced Button Inputs (`debounce()`):**

```
private boolean debounce(long lastPressTime) {
    return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
}

// ... later in controlServos() and other button-handling methods ...
if ((gamepad1.square || gamepad2.square) && debounce(lastSquareButtonPressTime)) {
    lastSquareButtonPressTime = System.currentTimeMillis();
    // ... servo action ...
}
```

Innovation: The `debounce()` method and the `last...PressTime` variables implement button debouncing. This is a crucial feature for reliable servo control in Teleop. Mechanical buttons can "bounce," sending multiple rapid signals for a single press. Debouncing ensures that each button press is registered only once, preventing unintended multiple servo activations for a single driver input. This makes the controls feel much more responsive and predictable.

**b) Servo Speed Multiplier and Joystick Control:**

```
private static final double SERVO_SPEED_MULTIPLIER = 0.3;
// ...
double rightStickY = gamepad1.right_stick_y != 0 ? gamepad1.right_stick_y : gamepad2.right_stick_y;
if (Math.abs(rightStickY) > 0.1) {
    clawShuCurrentPos = Math.max(0, Math.min(1, claw_shu.getPosition() - rightStickY * SERVO_SPEED_MULTIPLIER));
    claw_shu.setPosition(clawShuCurrentPos);
}
```

Innovation: The `SERVO_SPEED_MULTIPLIER` and the use of `gamepad1.right_stick_y` (and gamepad2's) for `claw_shu` servo control provide proportional manual servo control. Instead of just toggling servo positions with buttons, drivers can use the joystick to smoothly adjust the `claw_shu` servo's position with variable speed, allowing for fine-grained adjustments. The `SERVO_SPEED_MULTIPLIER` constant controls the sensitivity of this joystick control. This greatly enhances driver dexterity and control over complex mechanisms.

**c) Road Runner Actions for Complex Servo/Motor Sequences:**

```
public static class ServoAction implements Action { // ... }
public static class MotorAction implements Action { // ... }
public static class ArmMotorAction implements Action { // ... }
public static class SlideServoAction implements Action { // ... }

// Example usage in handleSlideMovement() (Chamber Auto Drive Routine):
Actions.runBlocking(
        drive.actionBuilder(drive.pose)
                .stopAndAdd(new ArmMotorAction((DcMotorEx) bigArmMotor, true, 0))
                .stopAndAdd(new ServoAction(backArmServo, BACK_ARM_RESET_POSITION))
                .stopAndAdd(new MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, 0))
                // ... more actions ...
                .build());
```

Innovation: The code defines custom Road Runner Action classes (`ServoAction`, `MotorAction`, `ArmMotorAction`, `SlideServoAction`) to encapsulate individual servo and motor movements. This is a highly innovative approach for Teleop:

- **Abstraction and Reusability:** These Action classes make the code much more modular and readable. Complex sequences of servo and motor commands can be built up in a clear, declarative way using `Actions.runBlocking()` and `.stopAndAdd()`.
- **Sequential Control:** Road Runner Actions are designed for sequential execution. This allows for building automated routines within Teleop that execute a series of precisely timed servo and motor movements.
- **Integration with Motion Planning:** Although not directly used for motion planning in this specific Teleop example, the use of Road Runner Actions demonstrates the potential to seamlessly integrate vision-guided motion with complex mechanism control routines in future iterations.

The "Chamber Auto Drive Routine" in `handleSlideMovement()` is a prime example of how these Actions can be used to create sophisticated, automated sequences within Teleop, triggered by a button press. This blurs the line between traditional Teleop and Autonomous, offering drivers powerful semi-autonomous capabilities.

### 3. Odometry and Navigation Innovations

**a) In-Code IMU Reset Routine (resetIMU()):**

```
private void resetIMU() {
    // ... (code to reset IMU and odometry) ...
}
```

Innovation: The `resetIMU()` method provides a driver-activated, in-code routine to reset the IMU and odometry heading. This is essential for competitive FTC:

- **Driver Control:** Drivers can reset the IMU at any point during a match, correcting for heading drift or misalignment that can accumulate over time or after robot collisions.
- **Robustness:** The routine includes checks to ensure the robot is stationary before resetting, improving the accuracy of the reset.
- **Telemetry Feedback:** Provides clear telemetry messages to the driver, indicating the status of the IMU reset process (resetting, success, failure).

**b) Pose Recording (handlePoseRecord()):**

```
private Pose2d recordedCatchPose = null;
private Pose2d recordedChamberPose = null;
// ...
private void handlePoseRecord() {
    if ((gamepad1.dpad_left || gamepad2.dpad_left) && debounce(lastDpadLeftPressTime)) {
        recordedCatchPose = drive.pose; // Record current pose
        // ...
    }
    // ... similar for chamber pose ...
}
```

Innovation: The `

handlePoseRecord()` method allows drivers to dynamically record robot poses during Teleop. This is a powerful feature for quickly creating and executing semi-autonomous routines without needing to pre-program precise coordinates:

- **On-the-Fly Programming:** Drivers can "teach" the robot specific locations on the field during the match itself.
- **Flexibility:** Adapt to changing field conditions or unexpected starting positions by re-recording poses as needed.

**c) Chamber Auto Drive Routine (Illustrative Example):**

```
if ((gamepad1.dpad_up || gamepad2.dpad_up) && debounce(lastDpadUpPressTime)) {
    // ... Chamber Auto Drive Routine ...
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // ... complex sequence of splines, servo actions, motor actions, wait states ...
            .build());
}
```

Innovation: The "Chamber Auto Drive Routine" (triggered by D-pad Up) demonstrates a sophisticated, pose-based autonomous sequence built directly within the Teleop code (though driver-activated).

- **Pose-to-Pose Navigation:** It uses Road Runner's `splineToSplineHeading` and `splineToConstantHeading` to navigate between recorded poses (`recordedCatchPose`, `recordedChamberPose`).
- **Integrated Mechanism Control:** It seamlessly integrates servo and motor actions (using the custom Action classes) within the motion sequence to control the arm, claw, and slides during navigation.

Illustrative Capability: Although commented out in the main `handleSlideMovement` loop for typical Teleop operation, this routine showcases the code's capability to execute complex, pre-defined autonomous sequences triggered by the driver during Teleop, greatly extending the robot's functionality beyond purely manual control.

## Adapting and Extending for Your Team

When adapting this competition-level Teleop code, consider:

- **Mechanism Customization:** Adapt the servo and motor control sections (e.g., `controlServos()`, `handleSlideMovement()`, `ServoAction`, `MotorAction`) to match your robot's specific mechanisms.
- **Teleop Control Scheme:** Customize the gamepad mappings in `controlClaw()`, `controlServos()`, `handleSlideMovement()`, and `driveRobot()` to create a driver control scheme that is intuitive and efficient for your drivers.
- **Autonomous Routine Expansion:** Further develop the "Chamber Auto Drive Routine" or create new driver-activated autonomous routines using the pose recording and Road Runner Action framework.
- **Vision Enhancement:** Continue to refine the `ColorDetectionPipelineImpl` for even greater robustness and accuracy.

## Conclusion

This 1500-line Teleop OpMode represents a significant advancement in FTC robot control, combining robust vision processing, advanced servo and motor control, precise navigation, and a driver-centric design