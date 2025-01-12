package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.GoBildaPinpointDriver;


@TeleOp(name = "Single Stick Mecanum Drive with Claw and Lift Control", group = "Drive")
public class SingleStick extends LinearOpMode {

    //region Constants and Configurations
    private static final double CLAW_INCREMENT = 0.4;
    private static final long DEBOUNCE_DELAY = 300;
    private static final double SLIDE_DOWN_POWER = -0.6;
    private static final double FRAME_HOLD_POSITION = 0.5;
    private static final double FRAME_INITIAL_POSITION = 0;
    private static final double CLAW_SHU_ROTATE_POSITION = 1.0;
    private static final double CLAW_SHU_INITIAL_POSITION = 0.0;
    private static final double SERVO_SPEED_MULTIPLIER = 0.02;
    private static final double FRAME_SERVO_SPEED = 0.03;
    private static final double DRIVE_STOP_THRESHOLD = 0.01;
    private static final double EPSILON = 0.01;

    private static final int SLIDE_HIGH = 2580;
    private static final int SLIDE_HOME = 0;
    private static final int SLIDE_MID = 1696;
    private static final int SLIDE_HANG = 250;
    private static final int SLIDE_HIGH_START = 2000;
    private static final int FRAME_MOVE_DELAY = 510;

    //endregion

    //region Enums
    private enum SlideState {
        IDLE,
        MOVING_TO_POSITION,
        MANUAL_DOWN
    }

    //endregion

    //region Hardware Declaration
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor Left_Hanging_Motor, Right_Hanging_Motor;
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame;
    //endregion

    //region State Variables
    private SlideState slideState = SlideState.IDLE;
    private boolean isClawOpen = false;
    private boolean isForwardClawOpen = false;
    private boolean isClawHengOpen = true;
    private boolean armForwardPosition = true;
    private boolean isClawShuRotating = false;
    private boolean isFrameMoving = false;
    private boolean isAutoSlideDown = false;
    private double clawPosition = 0.0;
    private double clawShuCurrentPos = 0.0;
    private double clawHengCurrentPos = 0.0;
    private double frameCurrentPosition = FRAME_INITIAL_POSITION;
    private long frameMoveStartTime = 0;

    private int leftBumperPressCount = 0;
    private double initialHeading = 0;
    private boolean isFirstReset = true;

    //endregion

    //region Timing and Debounce
    private long lastOptionButtonPressTime = 0;
    private long lastSquareButtonPressTime = 0;
    private long lastForwardButtonPressTime = 0;
    private long lastBackButtonPressTime = 0;
    private long lastCircleButtonPressTime = 0;
    private long lastTriangleButtonPressTime = 0;
    private long lastCrossButtonPressTime = 0;
    private long lastRightStickPressTime = 0;

    private ElapsedTime debounceTimer = new ElapsedTime();
    //endregion

    //region Odometry and Mecanum Drive
    private GoBildaPinpointDriver odo;
    private MecanumDrive drive;
    private Pose2d recordedPose = null;
    //endregion


    @Override
    public void runOpMode() {
        initialize();

        telemetry.addData("状态", "初始化完成");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pose = odo.getPosition();
            double robotHeading = (pose.getHeading(AngleUnit.DEGREES) + 360) % 360;

            driveRobot(robotHeading);
            controlClaw();
            controlServos();
            handleSlideMovement();
            handleFrame();
            handleClawShuControl();
            checkReset();
            handleLeftBumperAction();
            updateTelemetry(robotHeading);
        }
    }

    //region Initialization
    private void initialize() {
        initializeHardware();
        initializeServos();
        initializeOdometry();
        setInitialServoPositions();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        Left_Hanging_Motor = hardwareMap.get(DcMotor.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotor.class, "RightHangingMotor");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorRunMode(Left_Hanging_Motor, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunMode(Right_Hanging_Motor, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunMode(Left_Hanging_Motor, DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorRunMode(Right_Hanging_Motor, DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(Left_Hanging_Motor, DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroPowerBehavior(Right_Hanging_Motor, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeServos() {
        backgrap = hardwareMap.get(Servo.class, "backgrap");
        forward_slide = hardwareMap.get(Servo.class, "forward_slide");
        arm_forward = hardwareMap.get(Servo.class, "arm_forward");
        claw_shu = hardwareMap.get(Servo.class, "claw_shu");
        forward_claw = hardwareMap.get(Servo.class, "forward_claw");
        claw_heng = hardwareMap.get(Servo.class, "claw_heng");
        frame = hardwareMap.get(Servo.class, "frame");

        frame.setPosition(FRAME_INITIAL_POSITION);
        backgrap.setPosition(clawPosition);
        claw_heng.setPosition(0.55);
    }

    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "goBILDAPinpoint");
        odo.setOffsets(5.5, 121.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        resetIMU();
    }
    //endregion

    //region Initial Servo Positions
    private void setInitialServoPositions() {
        forward_slide.setPosition(0.88);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
    }
    //endregion

    //region Robot Control
    private void driveRobot(double robotHeading) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;

        double rotX = x * Math.cos(-Math.toRadians(robotHeading)) - y * Math.sin(-Math.toRadians(robotHeading));
        double rotY = x * Math.sin(-Math.toRadians(robotHeading)) + y * Math.cos(-Math.toRadians(robotHeading));

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
        drive.leftFront.setPower(frontLeftPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.rightBack.setPower(backRightPower);
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
        if (gamepad1.dpad_up && debounce(lastForwardButtonPressTime)) {
            lastForwardButtonPressTime = System.currentTimeMillis();
            forward_slide.setPosition(0.25);
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
            telemetry.addData("dpad_up pressed", "claw_shu pos: %.2f, isClawShuMoving: %b", claw_shu.getPosition(), isClawShuRotating);
            telemetry.update();
        }
        if (gamepad1.dpad_down && debounce(lastBackButtonPressTime)) {
            lastBackButtonPressTime = System.currentTimeMillis();
            forward_slide.setPosition(0.88);
            arm_forward.setPosition(0.8);
            claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
            claw_heng.setPosition(0.55);
            telemetry.addData("dpad_down pressed", "claw_shu pos: %.2f, isClawShuMoving: %b", claw_shu.getPosition(), isClawShuRotating);
            telemetry.update();
        }
        if (gamepad1.circle && debounce(lastCircleButtonPressTime)) {
            lastCircleButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(isForwardClawOpen ? 0.5 : 1);
            isForwardClawOpen = !isForwardClawOpen;
        }
        if (gamepad1.cross && debounce(lastCrossButtonPressTime)) {
            lastCrossButtonPressTime = System.currentTimeMillis();
            arm_forward.setPosition(armForwardPosition ? 0.15 : 0.8);
            armForwardPosition = !armForwardPosition;
        }
        if (gamepad1.triangle && debounce(lastTriangleButtonPressTime)) {
            lastTriangleButtonPressTime = System.currentTimeMillis();
            claw_heng.setPosition(isClawHengOpen ? 0.2 : 0);
            isClawHengOpen = !isClawHengOpen;
            telemetry.addData("Triangle pressed", "claw_heng pos: %.2f, isClawHengOpen: %b", claw_heng.getPosition(), isClawHengOpen);
            telemetry.update();
        }
    }
    //endregion

    //region Slide Control
    private void handleSlideMovement() {
        if (slideState == SlideState.IDLE) {
            setZeroPowerBehavior(Left_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
            setZeroPowerBehavior(Right_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
        }


        if (gamepad1.y) {
            telemetry.addData("滑轨状态", "已复位到零点");
        }
        else if (slideState == SlideState.IDLE) {

            if (gamepad1.left_bumper && !isClawShuRotating && leftBumperPressCount == 1) {
                handleLeftBumper();
                isAutoSlideDown = true;
            }
            else if (gamepad1.dpad_left) {
                setSlidePosition(SLIDE_MID);
                slideState = SlideState.MOVING_TO_POSITION;
            }
            else if (gamepad1.dpad_right) {
                setSlidePosition(SLIDE_HANG);
                slideState = SlideState.MOVING_TO_POSITION;
            }
            if (gamepad1.right_bumper) {
                setMotorRunMode(Left_Hanging_Motor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                setMotorRunMode(Right_Hanging_Motor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("滑轨电机模式", "切换到 RUN_WITHOUT_ENCODER (右肩键)");
                setMotorPower(Left_Hanging_Motor, SLIDE_DOWN_POWER);
                setMotorPower(Right_Hanging_Motor, SLIDE_DOWN_POWER);
                slideState = SlideState.MANUAL_DOWN;
            }
        }
        else if (slideState == SlideState.MOVING_TO_POSITION) {
            if (isSlideBusy()) {
                if (isAutoSlideDown && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH && !isFrameMoving) {
                    setSlidePosition(SLIDE_HOME);
                    slideState = SlideState.MOVING_TO_POSITION;
                    isAutoSlideDown = false;
                } else {
                    setZeroPowerBehavior(Left_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
                    setZeroPowerBehavior(Right_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
                    slideState = SlideState.IDLE;
                    setMotorPower(Left_Hanging_Motor,0);
                    setMotorPower(Right_Hanging_Motor,0);
                }
            }
        }
        else if (slideState == SlideState.MANUAL_DOWN) {
            if (!gamepad1.right_bumper) {
                setZeroPowerBehavior(Left_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
                setZeroPowerBehavior(Right_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
                setMotorPower(Left_Hanging_Motor,0);
                setMotorPower(Right_Hanging_Motor,0);
                slideState = SlideState.IDLE;

            }
        }
        else if (isSlideBusy()) {
            setZeroPowerBehavior(Left_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
            setZeroPowerBehavior(Right_Hanging_Motor,DcMotor.ZeroPowerBehavior.BRAKE);
            if(slideState == SlideState.MOVING_TO_POSITION){
                slideState = SlideState.IDLE;
            }
            setMotorPower(Left_Hanging_Motor,0);
            setMotorPower(Right_Hanging_Motor,0);

        }
        if (isClawShuRotating) {
            if (Math.abs(claw_shu.getPosition() - CLAW_SHU_ROTATE_POSITION) < EPSILON) {
                slideState = SlideState.MOVING_TO_POSITION;
                isClawShuRotating = false;
            }
        }
        if (slideState == SlideState.MOVING_TO_POSITION && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH_START && !isFrameMoving) {
            frameMoveStartTime = System.currentTimeMillis();
            frameCurrentPosition = FRAME_HOLD_POSITION;
            frame.setPosition(frameCurrentPosition);
            isFrameMoving = true;
        }
        if (isAutoSlideDown && slideState == SlideState.IDLE && !isFrameMoving) {
            setSlidePosition(SLIDE_HOME);
            slideState = SlideState.MOVING_TO_POSITION;
            isAutoSlideDown = false;
        }
    }

    private void setSlidePosition(int position) {
        setZeroPowerBehavior(Left_Hanging_Motor, DcMotor.ZeroPowerBehavior.FLOAT);
        setZeroPowerBehavior(Right_Hanging_Motor, DcMotor.ZeroPowerBehavior.FLOAT);

        Left_Hanging_Motor.setTargetPosition(position);
        Right_Hanging_Motor.setTargetPosition(position);
        setMotorRunMode(Left_Hanging_Motor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorRunMode(Right_Hanging_Motor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Left_Hanging_Motor, 1);
        setMotorPower(Right_Hanging_Motor, 1);
    }
    private boolean isSlideBusy() {
        return !Left_Hanging_Motor.isBusy() && !Right_Hanging_Motor.isBusy();
    }
    //endregion

    //region Frame Control
    private void handleFrame() {
        if (isFrameMoving) {
            if (frameCurrentPosition == FRAME_HOLD_POSITION && (System.currentTimeMillis() - frameMoveStartTime >= FRAME_MOVE_DELAY)) {
                isFrameMoving = false;
            }
        }
        if (!isFrameMoving) {
            frameCurrentPosition = Math.max(FRAME_INITIAL_POSITION, frameCurrentPosition - FRAME_SERVO_SPEED);
            frame.setPosition(frameCurrentPosition);
        }
    }
    //endregion

    //region Claw Shu Control
    private void handleClawShuControl() {
        double rightStickY = gamepad1.right_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        if (Math.abs(rightStickY) > 0.1) {
            clawShuCurrentPos = claw_shu.getPosition() - rightStickY * SERVO_SPEED_MULTIPLIER;
            clawShuCurrentPos = Math.max(0, Math.min(1, clawShuCurrentPos));
            claw_shu.setPosition(clawShuCurrentPos);
        }

        if (Math.abs(rightStickX) > 0.1) {
            clawHengCurrentPos = claw_heng.getPosition() + rightStickX * SERVO_SPEED_MULTIPLIER;
            clawHengCurrentPos = Math.max(0, Math.min(1, clawHengCurrentPos));
            claw_heng.setPosition(clawHengCurrentPos);
        }
        if (gamepad1.right_stick_button && debounce(lastRightStickPressTime)) {
            lastRightStickPressTime = System.currentTimeMillis();
            resetIMU();
        }
    }
    //endregion

    //region Reset Logic
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
        forward_claw.setPosition(1);
        isClawHengOpen = true;
        claw_heng.setPosition(0.55);
        armForwardPosition = true;
        arm_forward.setPosition(0.8);
        frameCurrentPosition = FRAME_INITIAL_POSITION;
        frame.setPosition(FRAME_INITIAL_POSITION);
        slideState = SlideState.IDLE;
        resetIMU();
        clawShuCurrentPos = CLAW_SHU_INITIAL_POSITION;
        claw_shu.setPosition(clawShuCurrentPos);
        clawHengCurrentPos = 0.55;
        claw_heng.setPosition(clawHengCurrentPos);
    }
    //endregion

    //region IMU Reset
    private void resetIMU() {
        stopDriveMotors();
        telemetry.addData("IMU", "正在重置,请保持机器人静止...");
        telemetry.update();
        if (isRobotStationary()) {
            if (isFirstReset) {
                initialHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
                odo.resetPosAndIMU(initialHeading + 180);
                isFirstReset = false;
                telemetry.addData("IMU", "第一次重置完成");
            } else {
                odo.resetPosAndIMU(initialHeading + 180);
                telemetry.addData("IMU", "重置完成");
            }
        } else {
            telemetry.addData("IMU", "重置失败,请确保机器人静止!");
        }
        telemetry.update();
    }

    private boolean isRobotStationary() {
        return Math.abs(leftFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(rightFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(leftBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(rightBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD;
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    //endregion

    //region Helper Methods
    private boolean debounce(long lastPressTime) {
        return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
    }

    private void setMotorRunMode(DcMotor motor, DcMotor.RunMode runMode) {
        if (motor.getMode() != runMode) {
            motor.setMode(runMode);
        }
    }
    private void setMotorPower(DcMotor motor,double power)
    {
        motor.setPower(power);
    }
    private void setZeroPowerBehavior(DcMotor motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if(motor.getZeroPowerBehavior()!=zeroPowerBehavior)
        {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
    //endregion

    //region Telemetry
    private void updateTelemetry(double robotHeading) {
        telemetry.addData("状态", "运行中");
        telemetry.addData("GoBildaIMU", "%.2f", robotHeading);
        telemetry.addData("滑轨", "左:%d, 右:%d", Left_Hanging_Motor.getCurrentPosition(), Right_Hanging_Motor.getCurrentPosition());
        telemetry.addData("前段", "滑轨:%.2f, 小臂:%.2f, 竖:%.2f, 横:%.2f, 前爪:%.2f",
                forward_slide.getPosition(),
                arm_forward.getPosition(),
                claw_shu.getPosition(),
                claw_heng.getPosition(),
                forward_claw.getPosition());
        telemetry.addData("后爪", "状态: %s, 位置:%.2f", isClawOpen ? "打开" : "关闭", clawPosition);
        telemetry.addData("框状态", "位置: %.2f", frame.getPosition());
        telemetry.addData("框舵机目标位置:", frameCurrentPosition);
        telemetry.addData("控制", "左肩=高位, D-pad左=中位, D-pad右=挂钩, 长按右肩=下降, Y=复位,右摇杆按钮=IMU重置");
        telemetry.addData("提示", "按下圆形按键控制前爪, 右摇杆控制前爪自由度,  三角键：前爪横向自由度， 交叉键：小臂");
        telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
    }
    //endregion

    //region Left Bumper Action
    private void handleLeftBumperAction() {
        if (gamepad1.left_bumper && debounceTimer.seconds() > 0.5) {
            debounceTimer.reset();
            leftBumperPressCount++;

            if (leftBumperPressCount == 1) {
                recordedPose = drive.pose;
                handleLeftBumper();
                telemetry.addData("Recorded Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                        recordedPose.position.x, recordedPose.position.y, Math.toDegrees(recordedPose.heading.toDouble()));
                telemetry.update();

            } else {

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(recordedPose, 0)
                                .build()
                );
                handleLeftBumper();
            }
        }
    }

    private void handleLeftBumper() {
        if (Math.abs(claw_shu.getPosition() - CLAW_SHU_INITIAL_POSITION) < EPSILON) {
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
            isClawShuRotating = true;
            telemetry.addData("left_bumper pressed", "claw_shu pos: %.2f, isClawShuMoving: %b", claw_shu.getPosition(), isClawShuRotating);
            telemetry.update();
        } else {
            setSlidePosition(SLIDE_HIGH);
            slideState = SlideState.MOVING_TO_POSITION;
        }
    }
    //endregion
}