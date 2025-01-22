package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "比赛用手动程序", group = "Competition")
public class SingleStickWithRR extends LinearOpMode {
    private MecanumDrive drive;
    private Pose2d recordedPoseBasket = null;
    private Pose2d recordedPoseChamber = null;
    private static final double CLAW_INCREMENT = 0.4, DEBOUNCE_DELAY = 200, SLIDE_DOWN_POWER = -0.6, FRAME_HOLD_POSITION = 0, FRAME_INITIAL_POSITION = 0.7, CLAW_SHU_ROTATE_POSITION = 1.0, CLAW_SHU_INITIAL_POSITION = 0.0, SERVO_SPEED_MULTIPLIER = 0.3, FRAME_SERVO_SPEED = 1, DRIVE_STOP_THRESHOLD = 0.01;
    private static final int SLIDE_HIGH = 2580, SLIDE_MID = 1350, SLIDE_HANG = 930, SLIDE_HOME = 0, SLIDE_HIGH_START = 2000;

    private enum SlideState { IDLE, MOVING_TO_POSITION, MANUAL_DOWN, HANGING_PAUSE }
    private SlideState slideState = SlideState.IDLE;

    private boolean isClawShuRotating = false, isFrameMoving = false, isAutoSlideDown = false, isClawOpen = false, isForwardClawOpen = false, isClawHengOpen = true, armForwardPosition = true, isFirstReset = true, isLeftBumperFirstPress = true;
    private double clawPosition = 0.0, clawShuCurrentPos = 0.0, clawHengCurrentPos = 0.0, frameCurrentPosition = FRAME_INITIAL_POSITION, clawShuInitPosition = 0.0, initialHeading = 0;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Left_Hanging_Motor, Right_Hanging_Motor;
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame;
    private GoBildaPinpointDriver odo;

    private long lastOptionButtonPressTime = 0, lastSquareButtonPressTime = 0, lastForwardButtonPressTime = 0, lastBackButtonPressTime = 0, lastCircleButtonPressTime = 0, lastTriangleButtonPressTime = 0, lastCrossButtonPressTime = 0, lastRightStickPressTime = 0, frameMoveStartTime = 0;
    private long hangStartTime = 0;


    @Override
    public void runOpMode() {
        initializeHardware();
        initializeServos();
        initializeOdometry();
        setInitialServoPositions();
        clawShuInitPosition = claw_shu.getPosition();
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
            handleFrame();
            handleClawShuControl();
            checkReset();
            updateTelemetry(robotHeading);
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
        double rotX = x * Math.cos(-Math.toRadians(robotHeading)) - y * Math.sin(-Math.toRadians(robotHeading)); //无头模式
        double rotY = x * Math.sin(-Math.toRadians(robotHeading)) + y * Math.cos(-Math.toRadians(robotHeading));
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
        rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
        leftBackDrive.setPower((rotY - rotX + rx) / denominator);
        rightBackDrive.setPower((rotY + rotX - rx) / denominator);
    }

    private void handleLeftBumper() {
        if (isLeftBumperFirstPress) { //记录位置
            recordedPoseBasket = drive.pose;
            telemetry.addData("Recorded Pose Basket", "X: %.2f, Y: %.2f, Heading: %.2f", recordedPoseBasket.position.x, recordedPoseBasket.position.y, Math.toDegrees(recordedPoseBasket.heading.toDouble()));
            isLeftBumperFirstPress = false;
        } else {  //自动驾驶到记录的位置
            Actions.runBlocking(drive.actionBuilder(drive.pose).splineToSplineHeading(recordedPoseBasket, 0).build());
            telemetry.addData("Current Pose Basket", "X: %.2f, Y: %.2f, Heading: %.2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble()));
        }
        telemetry.update();

        if (claw_shu.getPosition() == clawShuInitPosition) {
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
            isClawShuRotating = true;
        } else {
            setSlidePosition(SLIDE_HIGH);
            slideState = SlideState.MOVING_TO_POSITION;
        }
    }

    private void handleFrame() {
        if (isFrameMoving && frameCurrentPosition == FRAME_HOLD_POSITION && (System.currentTimeMillis() - frameMoveStartTime >= 490)) {
            isFrameMoving = false;
        }
        if (!isFrameMoving) {
            frameCurrentPosition = Math.max(FRAME_INITIAL_POSITION, frameCurrentPosition - FRAME_SERVO_SPEED);
            frame.setPosition(frameCurrentPosition);
        }
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
        isLeftBumperFirstPress = true;
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        Left_Hanging_Motor = hardwareMap.get(DcMotor.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotor.class, "RightHangingMotor");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        forward_claw.setPosition(1);
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
        forward_slide.setPosition(1);
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
        if (gamepad1.dpad_up && debounce(lastForwardButtonPressTime)) {
            lastForwardButtonPressTime = System.currentTimeMillis();
            forward_slide.setPosition(0.4);
            arm_forward.setPosition(0.4);
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
        }
        if (gamepad1.dpad_down && debounce(lastBackButtonPressTime)) {
            lastBackButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(0);
            sleep(250);
            forward_slide.setPosition(1);
            arm_forward.setPosition(0.8);
            claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
            claw_heng.setPosition(0.55);
        }

        if (gamepad1.circle && debounce(lastCircleButtonPressTime)) {
            lastCircleButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(isForwardClawOpen ? 0.5 : 1);
            isForwardClawOpen = !isForwardClawOpen;
        }
        // 处理交叉按钮逻辑
        if (gamepad1.cross && debounce(lastCrossButtonPressTime)) {
            lastCrossButtonPressTime = System.currentTimeMillis();
            if(armForwardPosition) {
                arm_forward.setPosition(0.15);
                armForwardPosition = false;
            } else {
                arm_forward.setPosition(0.4);
                armForwardPosition = true;
            }
        }


        if (gamepad1.triangle && debounce(lastTriangleButtonPressTime)) {
            lastTriangleButtonPressTime = System.currentTimeMillis();
            claw_heng.setPosition(isClawHengOpen ? 0.07 : 0.55);
            isClawHengOpen = !isClawHengOpen;
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
        return !Left_Hanging_Motor.isBusy() && !Right_Hanging_Motor.isBusy();
    }

    private void handleSlideMovement() {
        if (slideState == SlideState.IDLE) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.y) {
            telemetry.addData("滑轨状态", "已复位到零点");
        } else if (slideState == SlideState.IDLE) {
            if (gamepad1.left_bumper) {
                handleLeftBumper();
                isAutoSlideDown = true;
            } else if (gamepad1.dpad_left) {
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setSlidePosition(SLIDE_MID);
                slideState = SlideState.MOVING_TO_POSITION;

            } else if (gamepad1.dpad_right) {
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setSlidePosition(SLIDE_HANG);
                slideState = SlideState.MOVING_TO_POSITION;
            }

            if (gamepad1.right_bumper) {
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Left_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                Right_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                slideState = SlideState.MANUAL_DOWN;
            }

        } else if (slideState == SlideState.MOVING_TO_POSITION && isSlideBusy() ) {
            if(Left_Hanging_Motor.getTargetPosition() == SLIDE_HANG){
                hangStartTime = System.currentTimeMillis();
                backgrap.setPosition(0);
                slideState = SlideState.HANGING_PAUSE;
            } else if (Left_Hanging_Motor.getTargetPosition() == 0){
                slideState = SlideState.IDLE;
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
            }else if (Left_Hanging_Motor.getTargetPosition() == SLIDE_MID){
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideState = SlideState.IDLE;
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
            }

        } else if (slideState == SlideState.HANGING_PAUSE) {
            if (System.currentTimeMillis() - hangStartTime > 1000) {
                setSlidePosition(0);
                slideState = SlideState.MOVING_TO_POSITION;
            }
        }
        else if (slideState == SlideState.MANUAL_DOWN) {
            if (!gamepad1.right_bumper) {
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
            if (claw_shu.getPosition() == CLAW_SHU_ROTATE_POSITION) {
                sleep(300);
                setSlidePosition(SLIDE_HIGH);
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

    private boolean debounce(long lastPressTime) {
        return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
    }

    private void updateTelemetry(double robotHeading) {
        telemetry.addData("状态", "运行中");
        telemetry.addData("GoBildaIMU", "%.2f", robotHeading);
        telemetry.addData("滑轨", "左:%d, 右:%d", Left_Hanging_Motor.getCurrentPosition(), Right_Hanging_Motor.getCurrentPosition());
        telemetry.addData("前段", "滑轨:%.2f, 小臂:%.2f, 竖:%.2f, 横:%.2f, 前爪:%.2f", forward_slide.getPosition(), arm_forward.getPosition(), claw_shu.getPosition(), claw_heng.getPosition(), forward_claw.getPosition());
        telemetry.addData("后爪", "状态: %s, 位置:%.2f", isClawOpen ? "打开" : "关闭", clawPosition);
        telemetry.addData("框状态", "位置: %.2f", frame.getPosition());
        telemetry.addData("框舵机目标位置:", frameCurrentPosition);
        telemetry.addData("控制", "左肩=高位, D-pad左=中位, D-pad右=挂钩, 长按右肩=下降, Y=复位,右摇杆按钮=IMU重置");
        telemetry.addData("提示", "按下圆形按键控制前爪, 右摇杆控制前爪自由度,  三角键：前爪横向自由度， 交叉键：小臂");
        telemetry.update();
    }
}