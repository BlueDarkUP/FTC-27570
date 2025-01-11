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
    // 常量定义
    private static final double CLAW_INCREMENT = 0.4; // 爪子位置增量
    private static final long DEBOUNCE_DELAY = 300; // 防抖延时（毫秒）
    private static final double SLIDE_DOWN_POWER = -0.6; // 滑轨下降功率
    private static final double FRAME_HOLD_POSITION = 0.5; // 框舵机保持位置
    private static final double FRAME_INITIAL_POSITION = 0; // 框舵机初始位置
    private static final double CLAW_SHU_ROTATE_POSITION = 1.0; // 爪子旋转位置
    private static final double CLAW_SHU_INITIAL_POSITION = 0.0; // 爪子初始位置
    private static final double SERVO_SPEED_MULTIPLIER = 0.02; // 舵机控制速度
    private static final double FRAME_SERVO_SPEED = 0.03; // 框舵机控制速度
    private static final double DRIVE_STOP_THRESHOLD = 0.01; // 电机停止阈值
    private static final double EPSILON = 0.01; // 位置比较阈值

    // 滑轨位置常量
    private final int SLIDE_LOW = 0; // 滑轨低位
    private final int SLIDE_HIGH = 2580; // 滑轨高位

    // 滑轨状态枚举
    private enum SlideState {
        IDLE, // 空闲
        MOVING_TO_POSITION, // 移动到目标位置
        MANUAL_DOWN // 手动下降
    }

    private SlideState slideState = SlideState.IDLE; // 滑轨当前状态

    // 标志位
    private boolean isClawShuRotating = false; // 爪子是否在旋转
    private boolean isSlideMoving = false; // 滑轨是否在移动
    private boolean isFrameMoving = false; // 框舵机是否在移动
    private boolean isAutoSlideDown = false; // 是否自动下降
    private boolean isFirstReset = true; // 是否是第一次重置
    private double initialHeading = 0; // 初始航向

    // 电机和舵机
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor Left_Hanging_Motor, Right_Hanging_Motor;
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame;

    // 变量
    private boolean isClawOpen = false; // 爪子是否打开
    private double clawPosition = 0.0; // 爪子当前位置
    private boolean isForwardClawOpen = false; // 前爪是否打开
    private boolean isClawHengOpen = true; // 横向爪子是否打开
    private boolean armForwardPosition = true; // 小臂位置
    private double clawShuCurrentPos = 0.0; // 爪子旋转当前位置
    private double clawHengCurrentPos = 0.0; // 横向爪子当前位置
    private double frameCurrentPosition = FRAME_INITIAL_POSITION; // 框舵机当前位置
    private double clawShuInitPosition = 0.0; // 爪子旋转初始位置

    // 时间戳
    private long lastOptionButtonPressTime = 0;
    private long lastSquareButtonPressTime = 0;
    private long lastForwardButtonPressTime = 0;
    private long lastBackButtonPressTime = 0;
    private long lastCircleButtonPressTime = 0;
    private long lastTriangleButtonPressTime = 0;
    private long lastCrossButtonPressTime = 0;
    private long lastRightStickPressTime = 0;
    private long frameMoveStartTime = 0;

    // Odometry 对象
    private GoBildaPinpointDriver odo;
    private final boolean isHeadlessMode = true;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeServos();
        initializeOdometry();
        setInitialServoPositions();
        clawShuInitPosition = claw_shu.getPosition();

        telemetry.addData("状态", "初始化完成");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pose = odo.getPosition();
            double robotHeading = (pose.getHeading(AngleUnit.DEGREES) + 360) % 360;

            // 控制机器人移动
            driveRobot(robotHeading);
            // 控制爪子
            controlClaw();
            // 控制舵机
            controlServos();
            // 控制滑轨
            handleSlideMovement();
            // 控制框舵机
            handleFrame();
            // 控制爪子旋转
            handleClawShuControl();
            // 检查重置
            checkReset();
            // 更新 Telemetry
            updateTelemetry(robotHeading);
        }
    }

    // 初始化硬件
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

        // 设置电机模式
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // 初始化舵机
    private void initializeServos() {
        backgrap = hardwareMap.get(Servo.class, "backgrap");
        forward_slide = hardwareMap.get(Servo.class, "forward_slide");
        arm_forward = hardwareMap.get(Servo.class, "arm_forward");
        claw_shu = hardwareMap.get(Servo.class, "claw_shu");
        forward_claw = hardwareMap.get(Servo.class, "forward_claw");
        claw_heng = hardwareMap.get(Servo.class, "claw_heng");
        frame = hardwareMap.get(Servo.class, "frame");

        // 设置初始位置
        frame.setPosition(FRAME_INITIAL_POSITION);
        backgrap.setPosition(clawPosition);
        claw_heng.setPosition(0.55);
    }

    // 初始化 Odometry
    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "goBILDAPinpoint");
        odo.setOffsets(5.5, 121.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        resetIMU();
    }

    // 设置初始舵机位置
    private void setInitialServoPositions() {
        forward_slide.setPosition(0.88);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
    }

    // 控制机器人移动
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

    // 控制爪子
    private void controlClaw() {
        if (gamepad1.square && debounce(lastSquareButtonPressTime)) {
            lastSquareButtonPressTime = System.currentTimeMillis();
            clawPosition = isClawOpen ? Math.max(0.0, clawPosition - CLAW_INCREMENT) : Math.min(1.0, clawPosition + CLAW_INCREMENT);
            backgrap.setPosition(clawPosition);
            isClawOpen = !isClawOpen;
        }
    }

    // 控制舵机
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

    // 控制滑轨
    private void handleSlideMovement() {
        if (slideState == SlideState.IDLE) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // 滑轨初始位置
        int SLIDE_HOME = 0;
        if (gamepad1.y) {
            telemetry.addData("滑轨状态", "已复位到零点");
        } else if (slideState == SlideState.IDLE) {
            if (gamepad1.left_bumper && !isClawShuRotating) {
                handleLeftBumper();
                isAutoSlideDown = true;
            } else if (gamepad1.dpad_left) {
                int SLIDE_MID = 1696;
                setSlidePosition(SLIDE_MID);
                slideState = SlideState.MOVING_TO_POSITION;
            } else if (gamepad1.dpad_right) {
                int SLIDE_HANG = 250;
                setSlidePosition(SLIDE_HANG);
                slideState = SlideState.MOVING_TO_POSITION;
            }

            if (gamepad1.right_bumper) {
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("滑轨电机模式", "切换到 RUN_WITHOUT_ENCODER (右肩键)");
                Left_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                Right_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                slideState = SlideState.MANUAL_DOWN;
            }
        } else if (slideState == SlideState.MOVING_TO_POSITION) {
            if (isSlideBusy()) {
                if (isAutoSlideDown && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH && !isFrameMoving) {
                    setSlidePosition(SLIDE_HOME);
                    slideState = SlideState.MOVING_TO_POSITION;
                    isAutoSlideDown = false;
                } else {
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
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0);
                Right_Hanging_Motor.setPower(0);
                slideState = SlideState.IDLE;
                isSlideMoving = false;
            }
        } else if (isSlideBusy()) {
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
            if (Math.abs(claw_shu.getPosition() - CLAW_SHU_ROTATE_POSITION) < EPSILON) {
                slideState = SlideState.MOVING_TO_POSITION;
                isClawShuRotating = false;
                isSlideMoving = true;
            }
        }
        // 滑轨开始移动框的位置
        int SLIDE_HIGH_START = 2000;
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

    // 控制框舵机
    private void handleFrame() {
        if (isFrameMoving) {
            if (frameCurrentPosition == FRAME_HOLD_POSITION && (System.currentTimeMillis() - frameMoveStartTime >= 510)) {
                isFrameMoving = false;
            }
        }
        if (!isFrameMoving) {
            frameCurrentPosition = Math.max(FRAME_INITIAL_POSITION, frameCurrentPosition - FRAME_SERVO_SPEED);
            frame.setPosition(frameCurrentPosition);
        }
    }

    // 控制爪子旋转
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

    // 检查重置
    private void checkReset() {
        if (gamepad1.options && debounce(lastOptionButtonPressTime)) {
            lastOptionButtonPressTime = System.currentTimeMillis();
            resetAll();
        }
    }

    // 重置所有状态
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

    // 重置 IMU
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

    // 检查机器人是否静止
    private boolean isRobotStationary() {
        return Math.abs(leftFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(rightFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(leftBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(rightBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD;
    }

    // 停止驱动电机
    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // 设置滑轨位置
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
        isSlideMoving = true;
    }

    // 检查滑轨是否忙
    private boolean isSlideBusy() {
        return !Left_Hanging_Motor.isBusy() && !Right_Hanging_Motor.isBusy();
    }

    // 防抖检查
    private boolean debounce(long lastPressTime) {
        return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
    }

    // 更新 Telemetry
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

    // 处理左肩键按下事件
    private void handleLeftBumper() {
        if (Math.abs(claw_shu.getPosition() - clawShuInitPosition) < EPSILON) {
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
}