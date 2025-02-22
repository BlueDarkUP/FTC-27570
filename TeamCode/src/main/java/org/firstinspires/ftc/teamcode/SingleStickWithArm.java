package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.GoBildaPinpointDriver;


@TeleOp(name = "比赛用手动程序", group = "Competition")
public class SingleStickWithArm extends LinearOpMode {
    private MecanumDrive drive;
    private static final double CLAW_INCREMENT = 0.55;
    private static final double DEBOUNCE_DELAY = 250;
    private static final double BACK_ARM_RESET_POSITION = 0.18;
    private static final  int LIFT_UP_POSITION = 610;
    private static final double BACK_ARM_SET_POSITION = 0.84;
    private static final double SLIDE_DOWN_POWER = -0.6;
    private static final double FRAME_HOLD_POSITION = 0;
    private static final double FRAME_INITIAL_POSITION = 0.57;
    private static final double CLAW_SHU_ROTATE_POSITION = 1.0;
    private static final double CLAW_SHU_INITIAL_POSITION = 0.0;
    private static final double SERVO_SPEED_MULTIPLIER = 0.3;
    private static final double DRIVE_STOP_THRESHOLD = 0.01;
    private static final int SLIDE_HIGH = 2580, SLIDE_MID = 1550, SLIDE_HANG = 930, SLIDE_HOME = 0, SLIDE_HIGH_START = 2000;
    private static final int SLIDE_TOLERANCE = 100;
    private static final double SERVO_POSITION_TOLERANCE = 0.1;
    private static final int ENCODER_POSITION_TOLERANCE = 100;

    private Pose2d recordedCatchPose = null;
    private Pose2d recordedChamberPose = null;

    private enum SlideState {IDLE, MOVING_TO_POSITION, MANUAL_DOWN, HANGING_PAUSE}

    private SlideState slideState = SlideState.IDLE;
    private Servo backArmServo;
    private Servo backgrapServo;
    private DcMotor bigArmMotor;

    private boolean isClawShuRotating = false;
    private boolean isFrameMoving = false;
    private boolean isAutoSlideDown = false;
    private boolean isClawOpen = false;
    private boolean isForwardClawOpen = false;
    private boolean isClawHengOpen = true;
    private boolean armForwardPosition = true;
    private boolean isFirstReset = true;
    private double clawPosition = 0.0;
    private double clawShuCurrentPos = 0.0;
    private double clawHengCurrentPos = 0.0;
    private double frameCurrentPosition = FRAME_INITIAL_POSITION;
    private double initialHeading = 0;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Left_Hanging_Motor, Right_Hanging_Motor;
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame, forward_slide_2;
    private GoBildaPinpointDriver odo;

    private long lastOptionButtonPressTime = 0;
    private long lastSquareButtonPressTime = 0;
    private long lastCircleButtonPressTime = 0;
    private long lastTriangleButtonPressTime = 0;
    private long lastCrossButtonPressTime = 0;
    private long lastRightStickPressTime = 0;
    private long frameMoveStartTime = 0;
    private long lastDpadRightPressTime = 0;
    private long lastDpadLeftPressTime = 0;
    private long lastDpadUpPressTime = 0;
    private long lastDpadDownPressTime = 0;
    private long lastLeftBumperPressTime = 0;
    private long lastRightBumperPressTime = 0;

    private final double defaultServoPosition = 0.18;
    private final double defaultGrapServoPosition = 0.0;
    private final int targetMotorPosition = 387;
    private final int defaultMotorPosition = 0;
    private final int targetHangingMotorPosition = 610;
    private final int defaultHangingMotorPosition = 0;

    private boolean armHangingState = false;
    private boolean dpadLeftButtonPreviousState = false;
    private boolean motorsInBrake = true;
    private boolean isForwardSlideExtended = false;
    private boolean isDpadLeftUsed = false;
    private boolean isDpadRightUsed = false;

    private int rightBumperPressCount = 0; // Counter for right bumper presses
    private double driveSpeedMultiplier = 1.0; // Multiplier for drive speed
    private int dpadDownPressCount = 0; // Counter for D-pad down presses


    @Override
    public void runOpMode() {
        initializeHardware();
        initializeServos();
        initializeOdometry();
        setInitialServoPositions();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pose = odo.getPosition();
            drive.updatePoseEstimate();
            double robotHeading = (pose.getHeading(AngleUnit.DEGREES) + 360) % 360;

            driveRobot(robotHeading);
            controlClaw();
            controlServos();
            handleSlideMovement();
            handleClawShuControl();
            handlePoseRecord();
            checkReset();
            updateTelemetry(robotHeading);
        }
    }

    private void handlePoseRecord() {
        if (gamepad1.dpad_left && debounce(lastDpadLeftPressTime) && !isDpadLeftUsed) {
            lastDpadLeftPressTime = System.currentTimeMillis();
            recordedCatchPose = drive.pose;
            telemetry.addData("操作", "机器人位姿已设置为 catch pose");
            isDpadLeftUsed = true;
        }

        if (gamepad1.dpad_right && debounce(lastDpadRightPressTime) && !isDpadRightUsed) {
            lastDpadRightPressTime = System.currentTimeMillis();
            recordedChamberPose = drive.pose;
            telemetry.addData("操作", "已记录当前位姿为 chamber pose");
            isDpadRightUsed = true;
        }
    }

    private void handleClawShuControl() {
        double rightStickY = gamepad1.right_stick_y, rightStickX = gamepad1.right_stick_x;

        if (Math.abs(rightStickY) > 0.1) {
            clawShuCurrentPos = Math.max(0, Math.min(1, claw_shu.getPosition() - rightStickY * SERVO_SPEED_MULTIPLIER));
            claw_shu.setPosition(clawShuCurrentPos);
        }

        if (Math.abs(rightStickX) > 0.1) {
            clawHengCurrentPos = Math.max(0, Math.min(1, claw_heng.getPosition() + rightStickX * SERVO_SPEED_MULTIPLIER));
            claw_heng.setPosition(clawHengCurrentPos);
        }

        if (gamepad1.right_stick_button && debounce(lastRightStickPressTime)) {
            lastRightStickPressTime = System.currentTimeMillis();
            resetIMU();
        }
    }

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

    private boolean isRobotStationary() {
        return Math.abs(leftFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD && Math.abs(rightFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD && Math.abs(leftBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD && Math.abs(rightBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD;
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void driveRobot(double robotHeading) {
        double y = gamepad1.left_stick_y, x = -gamepad1.left_stick_x, rx = gamepad1.right_trigger - gamepad1.left_trigger;
        double rotX = x * Math.cos(-Math.toRadians(robotHeading)) - y * Math.sin(-Math.toRadians(robotHeading));
        double rotY = x * Math.sin(-Math.toRadians(robotHeading)) + y * Math.cos(-Math.toRadians(robotHeading));
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        leftFrontDrive.setPower(((rotY + rotX + rx) / denominator) * driveSpeedMultiplier);
        rightFrontDrive.setPower(((rotY - rotX - rx) / denominator) * driveSpeedMultiplier);
        leftBackDrive.setPower(((rotY - rotX + rx) / denominator) * driveSpeedMultiplier);
        rightBackDrive.setPower(((rotY + rotX - rx) / denominator) * driveSpeedMultiplier);
    }

    private void checkReset() {
        if (gamepad1.options && debounce(lastOptionButtonPressTime)) {
            lastOptionButtonPressTime = System.currentTimeMillis();
            resetAll();
        }
    }

    private void resetAll() {
        setInitialServoPositions();
        isClawOpen = false;
        clawPosition = 0.0;
        backgrap.setPosition(clawPosition);
        isForwardClawOpen = false;
        forward_claw.setPosition(0.88);
        isClawHengOpen = true;
        claw_heng.setPosition(0.55);
        armForwardPosition = true;
        arm_forward.setPosition(0.77);
        frameCurrentPosition = FRAME_INITIAL_POSITION;
        frame.setPosition(FRAME_INITIAL_POSITION);
        slideState = SlideState.IDLE;
        resetIMU();
        clawShuCurrentPos = CLAW_SHU_INITIAL_POSITION;
        claw_shu.setPosition(clawShuCurrentPos);
        clawHengCurrentPos = 0.55;
        claw_heng.setPosition(clawHengCurrentPos);
        armHangingState = false;
        dpadLeftButtonPreviousState = false;
        isForwardSlideExtended = false;
        isDpadLeftUsed = false;
        isDpadRightUsed = false;
        recordedCatchPose = null;
        recordedChamberPose = null;
        rightBumperPressCount = 0; // Reset speed control counter
        driveSpeedMultiplier = 1.0; // Reset speed multiplier to default
        dpadDownPressCount = 0; // Reset dpad down press counter
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        Left_Hanging_Motor = hardwareMap.get(DcMotor.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotor.class, "RightHangingMotor");
        backArmServo = hardwareMap.get(Servo.class, "back_arm");
        backgrapServo = hardwareMap.get(Servo.class, "backgrap");
        bigArmMotor = hardwareMap.get(DcMotor.class, "big_arm");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        bigArmMotor.setDirection(DcMotor.Direction.REVERSE);

        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bigArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bigArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorBrakeMode(true);

        backArmServo.setPosition(defaultServoPosition);
        backgrapServo.setPosition(defaultGrapServoPosition);

        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeServos() {
        backgrap = hardwareMap.get(Servo.class, "backgrap");
        forward_slide = hardwareMap.get(Servo.class, "forward_slide");
        forward_slide_2 = hardwareMap.get(Servo.class, "forward_slide_2");
        arm_forward = hardwareMap.get(Servo.class, "arm_forward");
        claw_shu = hardwareMap.get(Servo.class, "claw_shu");
        forward_claw = hardwareMap.get(Servo.class, "forward_claw");
        claw_heng = hardwareMap.get(Servo.class, "claw_heng");
        frame = hardwareMap.get(Servo.class, "frame");
        backArmServo = hardwareMap.get(Servo.class, "back_arm");
        backgrapServo = hardwareMap.get(Servo.class, "backgrap");
        bigArmMotor = hardwareMap.get(DcMotor.class, "big_arm");

        frame.setPosition(FRAME_INITIAL_POSITION);
        backgrap.setPosition(clawPosition);
        claw_heng.setPosition(0.55);
        forward_claw.setPosition(0.88);
        backArmServo.setPosition(defaultServoPosition);
        backgrapServo.setPosition(defaultGrapServoPosition);
    }

    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "goBILDAPinpoint");
        odo.setOffsets(5.5, 121.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        resetIMU();
        sleep(200);
    }

    private void setInitialServoPositions() {
        forward_slide.setPosition(0);
        forward_slide_2.setPosition(0);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
    }

    private void controlClaw() {
        if (gamepad1.square && debounce(lastSquareButtonPressTime)) {
            lastSquareButtonPressTime = System.currentTimeMillis();
            clawPosition = isClawOpen ? Math.max(0.0, clawPosition - CLAW_INCREMENT) : Math.min(1.0, clawPosition + CLAW_INCREMENT);
            backgrap.setPosition(clawPosition);
            isClawOpen = !isClawOpen;
        }
    }

    private void controlServos() {
        if (gamepad1.left_bumper && debounce(lastLeftBumperPressTime)
        ) {
            lastLeftBumperPressTime = System.currentTimeMillis();
            if (!isForwardSlideExtended) {
                forward_slide.setPosition(0.9);
                forward_slide_2.setPosition(0);
                arm_forward.setPosition(0.3);
                claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
                telemetry.addLine("--- Left Bumper Pressed - Extend ---");
                telemetry.addData("Arm Forward Position (Left Bumper Extend)", arm_forward.getPosition());
                telemetry.update();
                isForwardSlideExtended = true;
            } else {
                forward_claw.setPosition(0);
                sleep(250);
                forward_slide.setPosition(0);
                forward_slide_2.setPosition(0);
                arm_forward.setPosition(0.77);
                claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
                claw_heng.setPosition(0.55);
                telemetry.addLine("--- Left Bumper Pressed - Retract ---");
                telemetry.addData("Arm Forward Position (Left Bumper Retract)", arm_forward.getPosition());
                telemetry.update();
                isForwardSlideExtended = false;
            }
        }


        if (gamepad1.circle && debounce(lastCircleButtonPressTime)) {
            lastCircleButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(isForwardClawOpen ? 0.5 : 0.88);
            isForwardClawOpen = !isForwardClawOpen;
        }
        if (gamepad1.cross && debounce(lastCrossButtonPressTime)) {
            lastCrossButtonPressTime = System.currentTimeMillis();
            if (armForwardPosition) {
                arm_forward.setPosition(0.18);
                armForwardPosition = false;
                telemetry.addLine("--- Cross Button Pressed (Position 0.15) ---");
            } else {
                arm_forward.setPosition(0.3);
                armForwardPosition = true;
                telemetry.addLine("--- Cross Button Pressed (Position 0.4) ---");
            }
            telemetry.addData("Arm Forward Position (Cross)", arm_forward.getPosition());
            telemetry.addData("armForwardPosition Boolean", armForwardPosition);
            telemetry.update();
        }


        if (gamepad1.triangle && debounce(lastTriangleButtonPressTime)) {
            lastTriangleButtonPressTime = System.currentTimeMillis();
            claw_heng.setPosition(isClawHengOpen ? 0.07 : 0.55);
            isClawHengOpen = !isClawHengOpen;
        }

        if (gamepad1.right_bumper && debounce(lastRightBumperPressTime)) {
            lastRightBumperPressTime = System.currentTimeMillis();
            rightBumperPressCount++;
            if (rightBumperPressCount % 2 != 0) {
                driveSpeedMultiplier = 0.8;
                telemetry.addLine("--- Right Bumper Pressed - Speed Reduced to 0.5 ---");
            } else {
                driveSpeedMultiplier = 1.0;
                telemetry.addLine("--- Right Bumper Pressed - Speed Reset to 1.0 ---");
            }
            telemetry.addData("Drive Speed Multiplier", driveSpeedMultiplier);
            telemetry.update();

            armHangingState = !armHangingState;

            setMotorBrakeMode(false);

            double hangingMotorPower = 1.0;
            if (armHangingState) {
                Left_Hanging_Motor.setTargetPosition(targetHangingMotorPosition);
                Right_Hanging_Motor.setTargetPosition(targetHangingMotorPosition);
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Hanging_Motor.setPower(1);
                Right_Hanging_Motor.setPower(1);
                sleep(200);
                bigArmMotor.setTargetPosition(targetMotorPosition);
                double targetServoPosition = 0.84;
                backArmServo.setPosition(targetServoPosition);

                bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                double targetMotorPower = 0.7;
                bigArmMotor.setPower(targetMotorPower);


            } else {
                backArmServo.setPosition(defaultServoPosition);
                sleep(100);
                bigArmMotor.setTargetPosition(defaultMotorPosition);
                Left_Hanging_Motor.setTargetPosition(defaultHangingMotorPosition);
                Right_Hanging_Motor.setTargetPosition(defaultHangingMotorPosition);
                claw_shu.setPosition(0.32);

                bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                double defaultMotorPower = 0.35;
                bigArmMotor.setPower(defaultMotorPower);
                Left_Hanging_Motor.setPower(1);
                Right_Hanging_Motor.setPower(1);
            }

            if (opModeIsActive()) {
                setMotorBrakeMode(true);
            }
        }
    }

    private void setSlidePosition(int position) {
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Left_Hanging_Motor.setTargetPosition(position);
        Right_Hanging_Motor.setTargetPosition(position);
        if (Left_Hanging_Motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        Left_Hanging_Motor.setPower(1);
        Right_Hanging_Motor.setPower(1);
    }

    private boolean isSlideBusy() {
        int leftCurrentPos = Left_Hanging_Motor.getCurrentPosition();
        int rightCurrentPos = Right_Hanging_Motor.getCurrentPosition();
        int leftTargetPos = Left_Hanging_Motor.getTargetPosition();
        int rightTargetPos = Right_Hanging_Motor.getTargetPosition();

        int tolerance = SLIDE_TOLERANCE;

        return Math.abs(leftCurrentPos - leftTargetPos) <= tolerance && Math.abs(rightCurrentPos - rightTargetPos) <= tolerance;
    }

    private void handleSlideMovement() {
        if (slideState == SlideState.IDLE) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.y) {
            telemetry.addData("滑轨状态", "已复位到零点");
        } else if (slideState == SlideState.IDLE) {
            if (gamepad1.right_bumper && debounce(lastRightBumperPressTime)) {
                lastRightBumperPressTime = System.currentTimeMillis();
                armHangingState = !armHangingState;

                setMotorBrakeMode(false);

                double hangingMotorPower = 1.0;
                if (armHangingState) {
                    Left_Hanging_Motor.setTargetPosition(targetHangingMotorPosition);
                    Right_Hanging_Motor.setTargetPosition(targetHangingMotorPosition);
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Hanging_Motor.setPower(1);
                    Right_Hanging_Motor.setPower(1);
                    sleep(200);
                    bigArmMotor.setTargetPosition(targetMotorPosition);
                    double targetServoPosition = 0.87;
                    backArmServo.setPosition(targetServoPosition);

                    bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double targetMotorPower = 0.7;
                    bigArmMotor.setPower(targetMotorPower);


                } else {
                    backArmServo.setPosition(defaultServoPosition);
                    sleep(100);
                    bigArmMotor.setTargetPosition(defaultMotorPosition);
                    Left_Hanging_Motor.setTargetPosition(defaultHangingMotorPosition);
                    Right_Hanging_Motor.setTargetPosition(defaultHangingMotorPosition);
                    claw_shu.setPosition(0.32);

                    bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    double defaultMotorPower = 0.35;
                    bigArmMotor.setPower(defaultMotorPower);
                    Left_Hanging_Motor.setPower(1);
                    Right_Hanging_Motor.setPower(1);
                }

                if (opModeIsActive()) {
                    setMotorBrakeMode(true);
                }
            }


            if (gamepad1.right_trigger > 0.1) {
                setMotorBrakeMode(false);
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Left_Hanging_Motor.setPower(SLIDE_DOWN_POWER * gamepad1.right_trigger);
                Right_Hanging_Motor.setPower(SLIDE_DOWN_POWER * gamepad1.right_trigger);
                slideState = SlideState.MANUAL_DOWN;
            }


            if (gamepad1.dpad_up && debounce(lastDpadUpPressTime)) {
                lastDpadUpPressTime = System.currentTimeMillis();
                telemetry.addData("操作", "执行 Chamber Auto Drive...");
                telemetry.update();

                if (recordedCatchPose == null || recordedChamberPose == null) {
                    telemetry.addData("警告", "请先记录 catch pose 和 chamber pose");
                    telemetry.update();
                    return;
                }

                Pose2d current_chamber_pose = new Pose2d(recordedChamberPose.position.x, recordedChamberPose.position.y, recordedChamberPose.heading.toDouble());
                Vector2d CatchPoseInConstantHeading = new Vector2d(
                        recordedCatchPose.position.x - 0,
                        recordedCatchPose.position.y - 0
                );

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                //初次抓快构建三次样条曲线之前复位执行机构
                                .stopAndAdd(new ArmMotorAction((DcMotorEx) bigArmMotor, true, 0))
                                .stopAndAdd(new ServoAction(backArmServo, BACK_ARM_RESET_POSITION))
                                .stopAndAdd(new MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, 0))
                                .stopAndAdd(new ServoAction(backgrap, 0))
                                .splineToSplineHeading(recordedCatchPose, 0)
                                .waitSeconds(0.2)
                                .stopAndAdd(new ServoAction(backgrap, 0.6))
                                .build());

                int numberOfChamberPoints = 13;
                double yIncrement = 5 * 0.39370;

                for (int i = 0; i < numberOfChamberPoints; i++) {
                    Vector2d ChamberPoseInConstantHeading = new Vector2d(
                            current_chamber_pose.position.x - 0,
                            current_chamber_pose.position.y + 5 * 0.39370
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.PI)
                                    //在挂块之前伸出大臂，抬高滑轨，张开后爪
                                    .stopAndAdd(new MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, LIFT_UP_POSITION))
                                    .waitSeconds(0.1)
                                    .stopAndAdd(new ServoAction(backArmServo, BACK_ARM_SET_POSITION))
                                    .waitSeconds(0.1)
                                    .stopAndAdd(new ArmMotorAction((DcMotorEx) bigArmMotor, false, targetMotorPosition))
                                    .splineToConstantHeading(ChamberPoseInConstantHeading, Math.PI)//挂杆位置
                                    .waitSeconds(0.1)
                                    .stopAndAdd(new ServoAction(backgrap, 0))//打开后爪
                                    .waitSeconds(0.1)
                                    .setTangent(0)
                                    //在抓块之前复位
                                    .stopAndAdd(new ArmMotorAction((DcMotorEx) bigArmMotor, true, 0))
                                    .stopAndAdd(new ServoAction(backArmServo, BACK_ARM_RESET_POSITION))
                                    .stopAndAdd(new MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, 0))
                                    .splineToConstantHeading(CatchPoseInConstantHeading, 0)
                                    .waitSeconds(0.1)
                                    .stopAndAdd(new ServoAction(backgrap, 0.6))
                                    .build());


                    current_chamber_pose = new Pose2d(current_chamber_pose.position.x, current_chamber_pose.position.y + 5 * 0.39370 + yIncrement, current_chamber_pose.heading.toDouble());
                }


                telemetry.addData("操作", "Chamber Auto Drive 完成");
                telemetry.update();
            }

            if (gamepad1.dpad_down && debounce(lastDpadDownPressTime)) {
                lastDpadDownPressTime = System.currentTimeMillis();
                dpadDownPressCount++;
                setMotorBrakeMode(false);
                if (dpadDownPressCount % 2 != 0) { // Odd press
                    Left_Hanging_Motor.setTargetPosition(2620);
                    Right_Hanging_Motor.setTargetPosition(2620);
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Hanging_Motor.setPower(1);
                    Right_Hanging_Motor.setPower(1);
                    bigArmMotor.setTargetPosition(targetMotorPosition);
                    double targetServoPosition = 0.84;
                    backArmServo.setPosition(targetServoPosition);
                    bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double targetMotorPower = 0.7;
                    bigArmMotor.setPower(targetMotorPower);
                    telemetry.addData("D-pad Down", "Odd Press - Slide to Position 2620 and Arm Up");
                } else { // Even press
                    Left_Hanging_Motor.setTargetPosition(1100);
                    Right_Hanging_Motor.setTargetPosition(1100);
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Hanging_Motor.setPower(1);
                    Right_Hanging_Motor.setPower(1);
                    telemetry.addData("D-pad Down", "Even Press - Slide to Position 1100");
                }
                telemetry.update();
            }


        } else if (slideState == SlideState.MOVING_TO_POSITION && isSlideBusy()) {
            if (Left_Hanging_Motor.getTargetPosition() == SLIDE_HANG) {
                slideState = SlideState.IDLE;
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
                telemetry.addData("滑轨状态", "到达挂钩位置, 直接进入 IDLE");
                backgrap.setPosition(0);
            } else if (Left_Hanging_Motor.getTargetPosition() == 0) {
                slideState = SlideState.IDLE;
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
            } else if (Left_Hanging_Motor.getTargetPosition() == SLIDE_MID) {
                slideState = SlideState.IDLE;
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideState = SlideState.IDLE;
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
            }

        } else if (slideState == SlideState.HANGING_PAUSE) {
            long hangStartTime = 0;
            if (System.currentTimeMillis() - hangStartTime > 1000) {
                setSlidePosition(0);
                slideState = SlideState.MOVING_TO_POSITION;
            }
        } else if (slideState == SlideState.MANUAL_DOWN) {
            if (!(gamepad1.right_trigger > 0.1)) {
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
                slideState = SlideState.IDLE;
            }
        } else if (isSlideBusy()) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (slideState == SlideState.MOVING_TO_POSITION) {
                slideState = SlideState.IDLE;
            }
            Left_Hanging_Motor.setPower(0);
            Right_Hanging_Motor.setPower(0);
        }
        if (isClawShuRotating) {
            if (Math.abs(claw_shu.getPosition() - CLAW_SHU_ROTATE_POSITION) < SERVO_POSITION_TOLERANCE) {
                sleep(300);
                setSlidePosition(SLIDE_HIGH);
                slideState = SlideState.MOVING_TO_POSITION;
                isClawShuRotating = false;
            }
        }
        if (slideState == SlideState.MOVING_TO_POSITION &&
                Math.abs(Left_Hanging_Motor.getCurrentPosition() - SLIDE_HIGH_START) < ENCODER_POSITION_TOLERANCE && !isFrameMoving) {
            frameMoveStartTime = System.currentTimeMillis();
            frameCurrentPosition = FRAME_HOLD_POSITION;
            frame.setPosition(frameCurrentPosition);
            isFrameMoving = true;
        }
        if (isFrameMoving && slideState == SlideState.MOVING_TO_POSITION && (System.currentTimeMillis() - frameMoveStartTime >= 5000)) {
            setSlidePosition(SLIDE_HOME);
            slideState = SlideState.MOVING_TO_POSITION;
            isFrameMoving = false;
        }

        if (isAutoSlideDown && slideState == SlideState.IDLE && !isFrameMoving) {
            setSlidePosition(SLIDE_HOME);
            slideState = SlideState.MOVING_TO_POSITION;
            isAutoSlideDown = false;
        }
        dpadLeftButtonPreviousState = gamepad1.dpad_left;
    }

    private boolean debounce(long lastPressTime) {
        return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
    }

    private void updateTelemetry(double robotHeading) {
        telemetry.addData("状态", "运行中");
        telemetry.addData("---FTC27570遥测终端---",":");
        telemetry.addData("GoBildaIMU", "%.2f", robotHeading);
        telemetry.addData("滑轨", "左:%d, 右:%d", Left_Hanging_Motor.getCurrentPosition(), Right_Hanging_Motor.getCurrentPosition());
        telemetry.addData("前段", "滑轨:%.2f, 小臂:%.2f, 竖:%.2f, 横:%.2f, 前爪:%.2f", forward_slide.getPosition(), arm_forward.getPosition(), claw_shu.getPosition(), claw_heng.getPosition(), forward_claw.getPosition());
        telemetry.addData("后爪", "状态: %s, 位置:%.2f", isClawOpen ? "打开" : "关闭", clawPosition);
        telemetry.addData("框状态", "位置: %.2f", frame.getPosition());
        telemetry.addData("框舵机目标位置:", frameCurrentPosition);
        telemetry.addData("控制", "左肩=臂架切换, 右摇杆按钮=IMU重置, D-pad左=设置catch pose, D-pad右=记录chamber pose (一次性), D-pad上= Chamber Auto Drive, D-pad下=滑轨高度切换");
        telemetry.addData("提示", "按下圆形按键控制前爪, 右摇杆控制前爪自由度,  三角键：前爪横向自由度， 交叉键：小臂, 右肩键：前滑轨伸缩, 右扳机：滑轨下降");
        telemetry.addData("电机 big_arm", "位置: %d, 目标: %d, 忙碌: %b, 模式: %s, Brake: %b, Power: %.2f",
                bigArmMotor.getCurrentPosition(),
                armHangingState ? targetMotorPosition : defaultMotorPosition,
                bigArmMotor.isBusy(),
                bigArmMotor.getMode(),
                motorsInBrake,
                bigArmMotor.getPower());
        telemetry.addData("电机 LeftHangingMotor", "位置: %d, 目标: %d, 忙碌: %b, 模式: %s, Brake: %b",
                Left_Hanging_Motor.getCurrentPosition(),
                armHangingState ? targetHangingMotorPosition : defaultHangingMotorPosition,
                Left_Hanging_Motor.isBusy(),
                Left_Hanging_Motor.getMode(),
                motorsInBrake);
        telemetry.addData("电机 RightHangingMotor", "位置: %d, 目标: %d, 忙碌: %b, 模式: %s, Brake: %b",
                Right_Hanging_Motor.getCurrentPosition(),
                armHangingState ? targetHangingMotorPosition : defaultHangingMotorPosition,
                Right_Hanging_Motor.isBusy(),
                Right_Hanging_Motor.getMode(),
                motorsInBrake);
        telemetry.addData("舵机 back_arm", "%.2f", backArmServo.getPosition());
        telemetry.addData("舵机 backgrap", "%.2f", backgrapServo.getPosition());
        telemetry.addData("Arm Forward Servo", "Position: %.2f", arm_forward.getPosition());
        telemetry.addData("armForwardPosition Boolean", armForwardPosition);

        // 添加 catch pose 和 chamber pose 的 telemetry 输出
        telemetry.addData("Catch Pose", recordedCatchPose != null ? recordedCatchPose.toString() : "null");
        telemetry.addData("Chamber Pose", recordedChamberPose != null ? recordedChamberPose.toString() : "null");
        telemetry.addData("Drive Speed Multiplier", driveSpeedMultiplier); // Show speed multiplier in telemetry
        telemetry.addData("Dpad Down Press Count", dpadDownPressCount);

        telemetry.update();
    }
    private void setMotorBrakeMode(boolean brake) {
        DcMotor.ZeroPowerBehavior behavior = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        bigArmMotor.setZeroPowerBehavior(behavior);
        Left_Hanging_Motor.setZeroPowerBehavior(behavior);
        Right_Hanging_Motor.setZeroPowerBehavior(behavior);
        motorsInBrake = brake;
    }
    public class SlideServoAction implements Action{
        Servo S1 = null;
        Servo S2 = null;
        public SlideServoAction(Servo s1,Servo s2){
            this.S1 = s1;
            this.S2 = s2;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(S1.getPosition() <= 0.01&& S2.getPosition()>=0.54){
                S1.setPosition(0.55);
                S2.setPosition(0.0);
                return false;
            }
            S1.setPosition(0.0);
            S2.setPosition(0.55);
            return false;
        }
    }
    public static class ArmMotorAction implements Action{
        DcMotorEx arm = null;
        boolean IsSet = false;
        int position = 0;
        public ArmMotorAction(DcMotorEx M,boolean F, int p){
            this.arm = M;
            this.IsSet = F;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(IsSet){
                arm.setPower(0.5);
                return false;
            }
            arm.setPower(0.7);
            return false;
        }
    }
    public static class MotorAction implements Action {
        DcMotorEx Left_Hanging_Motor = null;
        DcMotorEx Right_Hanging_Motor = null;
        int position = 0;

        public MotorAction(DcMotorEx M1,DcMotorEx M2,int p){
            this.Left_Hanging_Motor = M1;
            this.Right_Hanging_Motor = M2;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Left_Hanging_Motor.setTargetPosition(position);
            Right_Hanging_Motor.setTargetPosition(position);
            Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Hanging_Motor.setPower(1);
            Right_Hanging_Motor.setPower(1);
            return false;
        }
    }
    public static class ServoAction implements Action {
        Servo FrontSlide = null;
        double position = 0;
        boolean hasInitialized = false;

        public ServoAction(Servo s, double p) {
            this.FrontSlide = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FrontSlide.setPosition(position);
            return false;
        }
    }
}