package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Single Stick Mecanum Drive with Claw and Lift Control", group = "Drive")
public class SingleStick extends LinearOpMode {
    // Constants for mechanism control
    private static final double CLAW_INCREMENT = 0.4;
    private static final long DEBOUNCE_DELAY = 200; // ms delay
    private static final double SLIDE_DOWN_POWER = -0.6;
    private static final double FRAME_HOLD_POSITION = 0.5; // 框保持的位置
    private static final double FRAME_INITIAL_POSITION = 0;
    private static final double CLAW_SHU_ROTATE_POSITION = 1.0; //claw_shu 旋转位置
    private static final double CLAW_SHU_INITIAL_POSITION = 0.0; // claw_shu 初始位置
    private static final double SERVO_SPEED_MULTIPLIER = 0.02; // 其他舵机控制速度
    private static final double FRAME_SERVO_SPEED = 0.03; // 框舵机控制速度 (提高)
    private static final double DRIVE_STOP_THRESHOLD = 0.01; // 用于静止检测的电机功率阈值

    // Slide Position Constants
    private final int SLIDE_LOW = 0;
    private final int SLIDE_HIGH = 2580;
    private final int SLIDE_HIGH_START = 2000; // 新增，滑轨开始移动 frame 的位置
    private final int SLIDE_HOME = 0;

    // Slide State
    private enum SlideState {
        IDLE,
        MOVING_UP,
        MOVING_UP_FIRST,
        MOVING_DOWN,
        MOVING_TO_POSITION,
        MANUAL_DOWN
    }

    private SlideState slideState = SlideState.IDLE;

    // Flags to indicate whether mechanisms are in motion
    private boolean isClawShuRotating = false;
    private boolean isSlideMoving = false;
    private boolean isFrameMoving = false; // 新增，用于跟踪 frame 舵机是否正在移动
    private boolean isAutoSlideDown = false; // 新增，用于标记是否自动下降

    // Motors for drive and hanging mechanism
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor Left_Hanging_Motor, Right_Hanging_Motor;

    // Servos for controlling claw and other mechanisms
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame;

    // Variables to manage claw and servo states
    private boolean isClawOpen = false;
    private double clawPosition = 0.0;
    private boolean isForwardClawOpen = false;
    private boolean isClawHengOpen = true;
    private boolean armForwardPosition = true;
    private double clawShuCurrentPos = 0.0;
    private double clawHengCurrentPos = 0.0;
    private double frameCurrentPosition = FRAME_INITIAL_POSITION; // 新增记录框舵机的位置
    private double clawShuInitPosition = 0.0;

    // Timing variables to manage button debounce
    private long lastOptionButtonPressTime = 0;
    private long lastSquareButtonPressTime = 0;
    private long lastForwardButtonPressTime = 0;
    private long lastBackButtonPressTime = 0;
    private long lastCircleButtonPressTime = 0;
    private long lastTriangleButtonPressTime = 0;
    private long lastCrossButtonPressTime = 0;
    private long lastRightStickPressTime = 0; // New variable for right stick button press

    private long frameMoveStartTime = 0; // 新增，记录 frame 舵机开始移动的时间

    // Pinpoint Odometry object
    private GoBildaPinpointDriver odo;
    private final boolean isHeadlessMode = true;

    // Flag to track if it's the first reset
    private boolean isFirstReset = true;
    private double initialHeading = 0;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeServos();
        initializeOdometry();
        setInitialServoPositions();
        clawShuInitPosition = claw_shu.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pose = odo.getPosition();
            double robotHeading = (pose.getHeading(AngleUnit.DEGREES) + 360) % 360;

            // Driving
            driveRobot(robotHeading);
            // Claw Controls
            controlClaw();
            // Servo Controls
            controlServos();
            // Slide Controls
            handleSlideMovement();
            //Frame Controls
            handleFrame();
            //Claw Shu Control
            handleClawShuControl();
            // Reset function
            checkReset();
            // Telemetry
            updateTelemetry(robotHeading);
        }
    }

    private void handleClawShuControl() {
        // Right stick control for claw_shu and claw_heng
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
        // Check for right stick button press
        if (gamepad1.right_stick_button && debounce(lastRightStickPressTime)) {
            lastRightStickPressTime = System.currentTimeMillis();
            // Reset IMU and set initial direction based on the flag
            resetIMU();
        }
    }

    private void resetIMU() {
        // Stop drive motors before resetting IMU
        stopDriveMotors();
        telemetry.addData("IMU", "正在重置,请保持机器人静止...");
        telemetry.update();
        // Check if the robot is stationary
        if(isRobotStationary()){
            if (isFirstReset) {
                // Get current IMU heading before resetting and add 180 degrees
                initialHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
                odo.resetPosAndIMU(initialHeading + 180); // Set to 180 degrees offset for initial reset
                isFirstReset = false; // After first reset, flag to false
                telemetry.addData("IMU", "第一次重置完成");
            } else {
                odo.resetPosAndIMU(initialHeading + 180); // Regular reset, 车头恢复正向
                telemetry.addData("IMU", "重置完成");
            }

        }else {
            telemetry.addData("IMU", "重置失败,请确保机器人静止!");
        }
        telemetry.update();
        // Add a short delay to allow user to see telemetry
        sleep(500);
    }

    private boolean isRobotStationary() {
        // Check if all drive motors are stopped or have very low power
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
    }

    private void handleLeftBumper() {
        if (claw_shu.getPosition() == clawShuInitPosition) {
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
            isClawShuRotating = true;
            telemetry.addData("left_bumper pressed", "claw_shu pos: %.2f, isClawShuMoving: %b", claw_shu.getPosition(), isClawShuRotating);
            telemetry.update();
        } else {
            setSlidePosition(SLIDE_HIGH);
            slideState = SlideState.MOVING_TO_POSITION;
            isSlideMoving = true;
        }
    }

    private void handleFrame() {
        if (isFrameMoving) {
            // 检查 frame 舵机是否到达目标位置，以及是否停留时间已过
            if (frameCurrentPosition == FRAME_HOLD_POSITION && (System.currentTimeMillis() - frameMoveStartTime >= 510)) {
                isFrameMoving = false; // 停止移动标志
            }
        }
        if (!isFrameMoving){
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
        slideState = SlideState.IDLE; // Reset slide state
        resetIMU(); // Reset IMU
        clawShuCurrentPos = CLAW_SHU_INITIAL_POSITION;
        claw_shu.setPosition(clawShuCurrentPos);
        clawHengCurrentPos = 0.55;
        claw_heng.setPosition(clawHengCurrentPos);
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

        // Set BRAKE mode initially
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set them back to RUN_USING_ENCODER mode - this is crucial
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
        //设置初始位置放在这里，无需等待框初始化，提高速度
        frame.setPosition(FRAME_INITIAL_POSITION);
        backgrap.setPosition(clawPosition);
        claw_heng.setPosition(0.55);
    }

    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "goBILDAPinpoint");
        //Initialize the settings for the GoBilda Odometry System:
        odo.setOffsets(5.5, 121.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Initial reset of IMU in init
        resetIMU();
        sleep(200); // Short delay
    }

    private void setInitialServoPositions() {
        forward_slide.setPosition(0.88);
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
        // 控制 forward_claw (圆形按钮)
        if (gamepad1.circle && debounce(lastCircleButtonPressTime)) {
            lastCircleButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(isForwardClawOpen ? 0.5 : 1);
            isForwardClawOpen = !isForwardClawOpen;
        }
        // 控制 arm_forward
        if (gamepad1.cross && debounce(lastCrossButtonPressTime)) {
            lastCrossButtonPressTime = System.currentTimeMillis();
            arm_forward.setPosition(armForwardPosition ? 0.15 : 0.8);
            armForwardPosition = !armForwardPosition;
        }
        //  控制 claw_heng
        if (gamepad1.triangle && debounce(lastTriangleButtonPressTime)) {
            lastTriangleButtonPressTime = System.currentTimeMillis();
            claw_heng.setPosition(isClawHengOpen ? 0.2 : 0);
            isClawHengOpen = !isClawHengOpen;
            telemetry.addData("Triangle pressed", "claw_heng pos: %.2f, isClawHengOpen: %b", claw_heng.getPosition(), isClawHengOpen);
            telemetry.addData("hanging motor", "left: %d, right: %d, isBusyLeft: %b, isBusyRight: %b", Left_Hanging_Motor.getCurrentPosition(), Right_Hanging_Motor.getCurrentPosition(), Left_Hanging_Motor.isBusy(), Right_Hanging_Motor.isBusy());
            telemetry.update();
        }
    }

    private void setSlidePosition(int position) {
        // Set to float mode before moving
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
        isSlideMoving = true;
    }

    private boolean isSlideBusy() {
        return !Left_Hanging_Motor.isBusy() && !Right_Hanging_Motor.isBusy();
    }

    private void handleSlideMovement() {
        // 如果滑轨处于空闲状态，将电机设置为 BRAKE 模式
        if (slideState == SlideState.IDLE) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.y) {
            telemetry.addData("滑轨状态", "已复位到零点");
        } else if (slideState == SlideState.IDLE) {
            if (gamepad1.left_bumper && !isClawShuRotating) {
                handleLeftBumper();
                isAutoSlideDown = true;  // 标记为自动下降模式
            } else if (gamepad1.dpad_left) {
                int SLIDE_MID = 1696;
                setSlidePosition(SLIDE_MID);
                slideState = SlideState.MOVING_TO_POSITION;
            } else if (gamepad1.dpad_right) {
                int SLIDE_HANG = 250;
                setSlidePosition(SLIDE_HANG);
                slideState = SlideState.MOVING_TO_POSITION;
            }

            if (gamepad1.right_bumper) { // 长按右肩键下降
                // 切换到 RUN_WITHOUT_ENCODER 模式
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("滑轨电机模式", "切换到 RUN_WITHOUT_ENCODER (右肩键)");
                Left_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                Right_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                slideState = SlideState.MANUAL_DOWN;
            }

        } else if (slideState == SlideState.MOVING_TO_POSITION) {
            if (isSlideBusy()) {
                // 如果是自动下降模式，并且滑轨已到达高位，且 frame 移动完成，则启动自动下降
                if (isAutoSlideDown && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH && !isFrameMoving) {
                    setSlidePosition(SLIDE_HOME); // 开始下降
                    slideState = SlideState.MOVING_TO_POSITION;
                    isAutoSlideDown = false; // 重置自动下降标志
                } else if (isAutoSlideDown && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH && isFrameMoving) {
                    // 如果自动下降标志为真，并且滑轨到达了高位，并且 frame 还在移动，不应该做任何事
                    //  如果 frame 移动已完成并且滑轨处于高位，启动自动下降
                } else {
                    // Brake the motors when they reach the target position
                    Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    slideState = SlideState.IDLE;
                    Left_Hanging_Motor.setPower(0);
                    Right_Hanging_Motor.setPower(0);
                    isSlideMoving = false;
                }
            }
        } else if (slideState == SlideState.MANUAL_DOWN) {
            if (!gamepad1.right_bumper) {
                // Brake the motors when releasing the right bumper
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
                slideState = SlideState.IDLE;
                isSlideMoving = false;
            }
        } else if (isSlideBusy()) {
            // Brake the motors if they finished running
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (slideState == SlideState.MOVING_TO_POSITION) {
                slideState = SlideState.IDLE;
            }
            Left_Hanging_Motor.setPower(0);
            Right_Hanging_Motor.setPower(0);
            isSlideMoving = false;
        }
        if (isClawShuRotating) {
            if (claw_shu.getPosition() == CLAW_SHU_ROTATE_POSITION) {
                sleep(300);
                setSlidePosition(SLIDE_HIGH);
                slideState = SlideState.MOVING_TO_POSITION;
                isClawShuRotating = false;
                isSlideMoving = true;
            }
        }
        // 在滑轨接近目标位置时启动 frame 舵机
        if (slideState == SlideState.MOVING_TO_POSITION && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH_START && !isFrameMoving) {
            frameMoveStartTime = System.currentTimeMillis();
            frameCurrentPosition = FRAME_HOLD_POSITION;
            frame.setPosition(frameCurrentPosition);
            isFrameMoving = true;
        }
        if (isAutoSlideDown && slideState == SlideState.IDLE && !isFrameMoving) {
            setSlidePosition(SLIDE_HOME); // 开始下降
            slideState = SlideState.MOVING_TO_POSITION;
            isAutoSlideDown = false; // 重置自动下降标志
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