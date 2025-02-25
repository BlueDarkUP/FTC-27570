# FTC Teleop Code for Competition - README (Innovation Focus)
[Fully automatic grasping visual program](https://github.com/BlueDarkUP/FTC-27570-INTO-THE-DEEP/blob/bc1q3yz7qs5dvm3l249rjqg393qfjddzev37cp84e3/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/WebcamExample.java)

## Introduction

This code is a sophisticated Teleop program designed for FTC competitions, incorporating advanced control features and optimizations for driver efficiency and robot performance. While primarily a Teleop program, it demonstrates innovative approaches in driver control, mechanism management, and system reset strategies.

This README will highlight the **innovative aspects** of this Teleop code, focusing on driver-centric control schemes, state-based mechanism management, and the integration of odometry and IMU for enhanced driver awareness.

## Core Innovations in Teleop Design

This Teleop code showcases several innovative design choices aimed at maximizing driver control and robot competitiveness:

1. **Single-Stick Drive with Robot-Centric Control & IMU Integration:** The code implements a single-stick driving scheme, simplifying driver input while maintaining precise robot control. It innovatively integrates IMU (Inertial Measurement Unit) and odometry to enable robot-centric driving, enhancing maneuverability and spatial awareness for the driver.

2. **State-Based Slide (Lift) Control:** The lifting mechanism (slide) is managed using a state machine (`SlideState` enum), providing a structured and robust way to control its movements. This state-based approach simplifies complex lift operations and ensures smooth transitions between different lift positions (High, Mid, Hang, Home, Manual Down).

3. **Debounced Button Inputs for Reliable Control:** The code extensively uses debouncing for button inputs (`debounce()` method), preventing accidental multiple triggers from a single button press. This debouncing mechanism is crucial for reliable and predictable control of robot mechanisms during fast-paced Teleop periods.

4. **Quick Reset and Calibration Features:** The code includes features for quick system reset (`resetAll()`) and IMU recalibration (`resetIMU()`) directly from the gamepad. These features are invaluable during matches for recovering from errors or re-calibrating the robot quickly without interrupting the Teleop period.

## Innovative Teleop Control and Mechanism Management

Let's examine the innovative components of the Teleop code in detail:

### 1. Single-Stick Drive with Robot-Centric Control & IMU Integration (`driveRobot()`, `resetIMU()`)

**a) Single-Stick Robot-Centric Drive (`driveRobot()`):**

```
private void driveRobot(double robotHeading) {
    double y = gamepad1.left_stick_y, x = -gamepad1.left_stick_x, rx = gamepad1.right_trigger - gamepad1.left_trigger;
    double rotX = x * Math.cos(-Math.toRadians(robotHeading)) - y * Math.sin(-Math.toRadians(robotHeading));
    double rotY = x * Math.sin(-Math.toRadians(robotHeading)) + y * Math.cos(-Math.toRadians(robotHeading));
    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
    rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
    leftBackDrive.setPower((rotY - rotX + rx) / denominator);
    rightBackDrive.setPower((rotY + rotX - rx) / denominator);
}
```

Innovation: The `driveRobot()` method implements a robot-centric drive control scheme using a single joystick (left stick of gamepad1) for translation and triggers for rotation. It's robot-centric because the drive directions are always relative to the robot's current heading, not a fixed field orientation.

- **IMU-Based Heading Compensation:** The code innovatively uses the `robotHeading` (derived from odometry and IMU) to perform a rotation transformation on the joystick inputs (x, y). `rotX` and `rotY` are calculated by rotating the joystick vector by the negative of the robot heading. This rotation effectively makes "forward" on the joystick always correspond to "forward relative to the robot's current direction", regardless of the robot's orientation on the field.

**b) IMU Reset and Recalibration (resetIMU()):**

```
private void resetIMU() {
    stopDriveMotors();
    telemetry.addData("IMU", "正在重置,请保持机器人静止...");
    telemetry.update();

    if (isRobotStationary()) {
        initialHeading = isFirstReset ? odo.getPosition().getHeading(AngleUnit.DEGREES) + 180 : initialHeading + 180;
        odo.resetPosAndIMU(initialHeading);
        isFirstReset = false;
        telemetry.addData("IMU", "重置完成");
    } else {
        telemetry.addData("IMU", "重置失败,请确保机器人静止!");
    }
    telemetry.update();
    sleep(500);
}
```

Innovation: The `resetIMU()` method provides a critical in-match IMU recalibration feature, triggered by `gamepad1.right_stick_button`. This is innovative because:

- **On-the-Fly Recalibration:** It allows the driver to quickly reset the IMU's heading during a match if the robot's orientation tracking drifts or becomes inaccurate. This is essential for maintaining accurate robot-centric control throughout the Teleop period.
- **Stationary Check:** The `isRobotStationary()` method ensures that the IMU is only reset when the robot is mostly stationary, improving the accuracy of the reset.
- **Odometry Integration:** The reset is performed through the `odo.resetPosAndIMU()` method, indicating a tight integration between odometry and IMU for pose tracking and recalibration.

### 2. State-Based Slide (Lift) Control (SlideState enum, handleSlideMovement(), setSlidePosition())

**a) SlideState Enum for Lift Management:**

```java
private enum SlideState {IDLE, MOVING_TO_POSITION, MANUAL_DOWN, HANGING_PAUSE}
private SlideState slideState = SlideState.IDLE;
```

Innovation: The `SlideState` enum and the `slideState` variable implement a state machine for controlling the lift mechanism. This is an innovative approach in Teleop because it provides a structured way to manage complex lift operations and transitions, making the lift control more robust and predictable. The states are:

- **IDLE:** Lift is at rest, motors braked.
- **MOVING_TO_POSITION:** Lift is actively moving to a target encoder position.
- **MANUAL_DOWN:** Lift is being manually lowered by the driver (right bumper held).
- **HANGING_PAUSE:** A brief pause state (not fully implemented in behavior, but part of the state structure).

**b) State-Driven Lift Movement (handleSlideMovement()):**

```java
private void handleSlideMovement() {
    // ... (state machine logic using switch/if-else based on slideState) ...
}
```

Innovation: The `handleSlideMovement()` method contains the state machine logic. It checks the `slideState` and gamepad inputs to determine the lift's behavior in each state. This state-driven approach is innovative in Teleop because it:

- **Simplifies Complex Sequences:** It breaks down complex lift operations (like moving to high, mid, hang positions, manual down) into manageable states and transitions.
- **Improves Predictability:** The state machine ensures that the lift behaves predictably in response to driver inputs and internal conditions (like reaching a target position).
- **Enhances Robustness:** State machines are inherently more robust to unexpected inputs or conditions compared to purely reactive or sequential code.

**c) Encoder-Based Position Control (setSlidePosition(), isSlideBusy()):**

```
private void setSlidePosition(int position) {
    // ... (code to set target position and motor mode) ...
}

private boolean isSlideBusy() {
    // ... (code to check if lift is still moving towards target) ...
}
```

Innovation: The lift control is encoder-based, using `setSlidePosition()` to move the lift to specific encoder positions (SLIDE_HIGH, SLIDE_MID, SLIDE_HANG, SLIDE_HOME). `isSlideBusy()` checks if the lift is still in motion towards its target, considering a `SLIDE_TOLERANCE` for practical position accuracy. Encoder-based control in Teleop provides:

- **Precise Positioning:** The lift moves to consistent, pre-defined heights, improving repeatability for scoring or manipulation tasks.
- **Motor Load Management:** `setSlidePosition()` sets the motors to `RUN_TO_POSITION` mode, which can help manage motor load and prevent overheating compared to purely power-based control, especially during extended Teleop periods.

### 3. Debounced Button Inputs (debounce() method):

```
private boolean debounce(long lastPressTime) {
    return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
}
```

Innovation: The `debounce()` method is a utility function used throughout the code to debounce button presses. This is innovative in Teleop for ensuring reliable and clean button input handling.

- **Prevents Accidental Multiple Triggers:** Debouncing prevents a single physical button press from being registered as multiple button presses by the robot's control system. This is crucial because mechanical buttons can sometimes "bounce" electrically, sending multiple signals for a single press.
- **Enhanced Driver Control:** Debouncing makes the button controls more predictable and easier to use for the driver, as actions are only triggered once per intended button press, even if the driver presses quickly or holds the button down.

### 4. Quick Reset and Calibration Features (resetAll(), resetIMU(), checkReset())

**a) Full System Reset (resetAll(), checkReset()):**

```
private void checkReset() {
    if (gamepad1.options && debounce(lastOptionButtonPressTime)) {
        lastOptionButtonPressTime = System.current

TimeMillis();
        resetAll();
    }
}

private void resetAll() {
    // ... (code to reset servos, motors, states, IMU, etc.) ...
}
```

Innovation: The `resetAll()` method, triggered by `gamepad1.options` (debounced), provides a comprehensive system reset feature. This is innovative for in-match recovery because it:

- **Quick Recovery from Errors:** It allows the driver to quickly reset the entire robot system to a known initial state if something goes wrong during a match (e.g., mechanisms get stuck, pose tracking is lost).
- **Resets All Mechanisms and States:** `resetAll()` resets servos to initial positions, motors to zero power, the slide state machine to `IDLE`, and crucially, also calls `resetIMU()` to recalibrate the IMU. This comprehensive reset ensures a clean restart.

**b) IMU-Specific Reset (resetIMU()):** The `resetIMU()` method itself (described in section 1b) is also part of the quick calibration/reset features. It allows for IMU recalibration independently of a full system reset.

## Adapting and Extending for Your Team

When adapting this Teleop code, consider these points to leverage its innovations and tailor it to your robot:

- **Single-Stick Drive Tuning:** Experiment with the drive parameters and joystick scaling in `driveRobot()` to fine-tune the single-stick drive feel for your driver's preferences. You might want to adjust the sensitivity or add exponential scaling.
- **State Machine Expansion:** Extend the `SlideState` state machine to manage other complex mechanisms on your robot using similar state-based control. State machines are