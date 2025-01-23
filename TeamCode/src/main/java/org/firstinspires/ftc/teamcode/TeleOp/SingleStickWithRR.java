//所有注释由Gemini 2.0 Flash Thinking Experimental 01-21模型生成


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

/**
 * @TeleOp(name = "比赛用手动程序", group = "Competition")
 *  此注释标记此类为一个 TeleOp 程序，它将出现在 FTC 驱动站应用程序中，
 *  名称为 "比赛用手动程序"，并被归类在 "Competition" 组下。
 *  此程序设计用于比赛期间机器人的手动控制。
 */
@TeleOp(name = "比赛用手动程序", group = "Competition")
public class SingleStickWithRR extends LinearOpMode {
    //region 硬件和 Road Runner 设置

    private MecanumDrive drive; // Road Runner 麦克纳姆轮驱动类，用于自主运动
    private Pose2d recordedPoseBasket = null; // 存储记录篮筐位置时机器人的姿态 (使用左肩键记录)
    private Pose2d recordedPoseChamber = null; // 存储记录料仓位置时机器人的姿态 (当前代码中未使用)

    //endregion

    //region 常量声明

    // 常量用于舵机增量、防抖延迟、滑轨动力、舵机位置和驱动阈值
    private static final double CLAW_INCREMENT = 0.4, // 后爪舵机位置调整的增量
            DEBOUNCE_DELAY = 200, // 防抖延迟，单位毫秒，防止按钮连按
            SLIDE_DOWN_POWER = -0.6, // 手动下拉滑轨的动力
            FRAME_HOLD_POSITION = 0, // 框架舵机保持位置
            FRAME_INITIAL_POSITION = 0.7, // 框架舵机初始位置
            CLAW_SHU_ROTATE_POSITION = 1.0, // 竖爪舵机旋转位置
            CLAW_SHU_INITIAL_POSITION = 0.0, // 竖爪舵机初始位置
            SERVO_SPEED_MULTIPLIER = 0.3, // 使用摇杆控制舵机速度的乘数
            FRAME_SERVO_SPEED = 1, // 框架舵机自动移动的速度
            DRIVE_STOP_THRESHOLD = 0.01; // 判定机器人静止的驱动阈值

    // 常量用于滑轨电机的目标位置，单位为编码器计数
    private static final int SLIDE_HIGH = 2580, // 高位滑轨高度的目标位置
            SLIDE_MID = 1350, // 中位滑轨高度的目标位置
            SLIDE_HANG = 930, // 挂钩滑轨高度的目标位置
            SLIDE_HOME = 0, // 归位/底部滑轨高度的目标位置
            SLIDE_HIGH_START = 2000; // 当提升到高位时，开始移动框架舵机的编码器计数阈值

    //endregion

    //region 枚举和状态变量

    /**
     * 枚举类型，表示滑轨机构的不同状态。
     * 用于线性滑轨的状态机控制。
     */
    private enum SlideState { IDLE, MOVING_TO_POSITION, MANUAL_DOWN, HANGING_PAUSE }
    private SlideState slideState = SlideState.IDLE; // 滑轨机构的当前状态，初始化为 IDLE (空闲)

    // 布尔标志，用于跟踪各种状态和条件
    private boolean isClawShuRotating = false, // 标志，指示竖爪舵机是否正在旋转
            isFrameMoving = false, // 标志，指示框架舵机是否正在自动移动
            isAutoSlideDown = false, // 标志，指示自动滑轨下降是否激活
            isClawOpen = false, // 标志，跟踪后爪是否打开或关闭
            isForwardClawOpen = false, // 标志，跟踪前爪是否打开或关闭
            isClawHengOpen = true, // 标志，跟踪横爪是否打开或关闭
            armForwardPosition = true, // 标志，跟踪前臂舵机的位置 (true 表示初始/抬起位置)
            isFirstReset = true, // 标志，指示是否是程序启动后的第一次 IMU 重置
            isLeftBumperFirstPress = true; // 标志，跟踪左肩键是否第一次按下 (用于姿态记录)

    // Double 类型变量，用于存储舵机位置和其他值
    private double clawPosition = 0.0, // 后爪舵机的当前位置 (0.0 到 1.0)
            clawShuCurrentPos = 0.0, // 竖爪舵机的当前位置
            clawHengCurrentPos = 0.0, // 横爪舵机的当前位置
            frameCurrentPosition = FRAME_INITIAL_POSITION, // 框架舵机的当前位置
            clawShuInitPosition = 0.0, // 竖爪舵机的初始位置，在初始化时捕获
            initialHeading = 0; // IMU 重置后机器人的初始航向角

    //endregion

    //region 硬件声明

    // DC 电机对象，用于驱动电机和挂钩电机
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Left_Hanging_Motor, Right_Hanging_Motor;

    // 舵机对象，用于各种机器人机构 (爪子、滑轨、手臂、框架)
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame;

    // 里程计对象，用于机器人定位
    private GoBildaPinpointDriver odo;

    //endregion

    //region 时间跟踪变量

    // 变量，用于跟踪上次按钮按下时间，实现防抖功能
    private long lastOptionButtonPressTime = 0,
            lastSquareButtonPressTime = 0,
            lastForwardButtonPressTime = 0,
            lastBackButtonPressTime = 0,
            lastCircleButtonPressTime = 0,
            lastTriangleButtonPressTime = 0,
            lastCrossButtonPressTime = 0,
            lastRightStickPressTime = 0,
            frameMoveStartTime = 0; // 框架舵机开始移动的时间戳
    private long hangStartTime = 0; // 挂钩暂停的开始时间戳

    //endregion

    /**
     * @Override
     * @method runOpMode()
     * 这是 TeleOp 程序的主入口点。当在 FTC 驱动站应用程序上按下 "INIT" 然后 "START" 按钮时执行。
     */
    @Override
    public void runOpMode() {
        initializeHardware(); // 初始化电机和传感器
        initializeServos(); // 初始化舵机对象
        initializeOdometry(); // 初始化里程计系统
        setInitialServoPositions(); // 在程序开始时将舵机设置到初始位置
        clawShuInitPosition = claw_shu.getPosition(); // 捕获竖爪舵机的初始位置
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); // 初始化 Road Runner 麦克纳姆轮驱动，初始姿态为 (0,0,0)

        telemetry.addData("Status", "Initialized"); // 发送初始化状态到遥测
        telemetry.update(); // 更新遥测，在驱动站上显示

        waitForStart(); // 等待按下 "START" 按钮

        while (opModeIsActive()) { // 主循环，持续运行直到按下 "STOP" 按钮
            odo.update(); // 更新里程计读数
            Pose2D pose = odo.getPosition(); // 从里程计获取当前姿态 (位置和航向角)
            drive.updatePoseEstimate(); // 基于里程计更新 Road Runner 的内部姿态估计
            double robotHeading = (pose.getHeading(AngleUnit.DEGREES) + 360) % 360; // 获取机器人航向角，单位为度，归一化到 0-360

            driveRobot(robotHeading); // 基于手柄输入和机器人航向角控制机器人驱动
            controlClaw(); // 基于手柄输入控制后爪
            controlServos(); // 基于手柄输入控制其他舵机 (前爪、滑轨臂等)
            handleSlideMovement(); // 基于手柄输入和状态机处理线性滑轨运动
            handleFrame(); // 在滑轨运动期间自动控制框架舵机
            handleClawShuControl(); // 使用右摇杆控制竖爪和横爪舵机
            checkReset(); // 检查复位命令 (选项按钮)
            updateTelemetry(robotHeading); // 更新遥测数据，在驱动站上显示
        }
    }

    /**
     * @method handleClawShuControl()
     * 使用 gamepad1 的右摇杆控制竖爪 (claw_shu) 和横爪 (claw_heng) 舵机。
     * 并在按下右摇杆按钮时处理 IMU 重置。
     */
    private void handleClawShuControl() {
        double rightStickY = gamepad1.right_stick_y, // 获取右摇杆垂直输入 (-1 到 1)
                rightStickX = gamepad1.right_stick_x; // 获取右摇杆水平输入 (-1 到 1)

        // 使用右摇杆 Y 轴控制竖爪舵机 (claw_shu)
        if (Math.abs(rightStickY) > 0.1) { // 检查摇杆输入是否足够显著
            clawShuCurrentPos = Math.max(0, Math.min(1, claw_shu.getPosition() - rightStickY * SERVO_SPEED_MULTIPLIER)); // 计算新位置，钳制在 0-1 范围内
            claw_shu.setPosition(clawShuCurrentPos); // 将竖爪舵机设置到新位置
        }

        // 使用右摇杆 X 轴控制横爪舵机 (claw_heng)
        if (Math.abs(rightStickX) > 0.1) { // 检查摇杆输入是否足够显著
            clawHengCurrentPos = Math.max(0, Math.min(1, claw_heng.getPosition() + rightStickX * SERVO_SPEED_MULTIPLIER)); // 计算新位置，钳制在 0-1 范围内
            claw_heng.setPosition(clawHengCurrentPos); // 将横爪舵机设置到新位置
        }

        // 使用右摇杆按钮按下进行 IMU 重置
        if (gamepad1.right_stick_button && debounce(lastRightStickPressTime)) { // 检查右摇杆按钮是否按下并防抖
            lastRightStickPressTime = System.currentTimeMillis(); // 更新上次按下时间
            resetIMU(); // 调用方法重置 IMU 和里程计
        }
    }

    /**
     * @method resetIMU()
     * 重置 IMU (惯性测量单元) 和里程计系统。
     * 它会停止驱动电机，检查机器人是否静止，然后重置 IMU 和里程计。
     */
    private void resetIMU() {
        stopDriveMotors(); // 停止所有驱动电机，确保机器人静止
        telemetry.addData("IMU", "正在重置,请保持机器人静止..."); // 发送遥测消息，指示 IMU 正在重置
        telemetry.update(); // 更新遥测，显示消息

        if (isRobotStationary()) { // 检查机器人是否被认为静止 (基于电机动力)
            initialHeading = isFirstReset ? odo.getPosition().getHeading(AngleUnit.DEGREES) + 180 : initialHeading + 180; // 计算新的初始航向角，每次重置增加 180 度
            odo.resetPosAndIMU(initialHeading); // 使用计算出的初始航向角重置里程计和 IMU
            isFirstReset = false; // 首次重置后设置标志为 false
            telemetry.addData("IMU", "重置完成"); // 发送遥测消息，指示 IMU 重置成功
        } else {
            telemetry.addData("IMU", "重置失败,请确保机器人静止!"); // 发送遥测消息，指示 IMU 重置失败，因为机器人正在移动
        }
        telemetry.update(); // 更新遥测，显示消息
        sleep(500); // 暂停 500 毫秒，以便显示遥测消息
    }

    /**
     * @method isRobotStationary()
     * 基于驱动电机的动力检查机器人是否静止。
     * @return boolean - 如果所有驱动电机动力都低于 DRIVE_STOP_THRESHOLD，则返回 true，表示机器人静止。
     */
    private boolean isRobotStationary() {
        return Math.abs(leftFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(rightFrontDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(leftBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD &&
                Math.abs(rightBackDrive.getPower()) <= DRIVE_STOP_THRESHOLD;
    }

    /**
     * @method stopDriveMotors()
     * 将所有驱动电机的动力设置为 0，有效地停止机器人的运动。
     */
    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * @method driveRobot(double robotHeading)
     * 基于手柄输入和当前机器人航向角控制机器人的麦克纳姆轮驱动。
     * 实现场地方向控制，意味着机器人相对于场地移动，而不是相对于自身方向。
     * @param robotHeading - 机器人当前的航向角，单位为度，用于场地方向计算。
     */
    private void driveRobot(double robotHeading) {
        double y = gamepad1.left_stick_y, // 垂直手柄输入 (-1 到 1)
                x = -gamepad1.left_stick_x, // 水平手柄输入 (-1 到 1)，取反以获得正确的方向
                rx = gamepad1.right_trigger - gamepad1.left_trigger; // 旋转手柄输入 (右扳机 - 左扳机)

        // 场地方向驱动计算：基于机器人航向角旋转摇杆输入
        double rotX = x * Math.cos(-Math.toRadians(robotHeading)) - y * Math.sin(-Math.toRadians(robotHeading)); // 旋转后的 X 分量
        double rotY = x * Math.sin(-Math.toRadians(robotHeading)) + y * Math.cos(-Math.toRadians(robotHeading)); // 旋转后的 Y 分量
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // 归一化动力值，使其在 -1 到 1 范围内

        // 基于计算出的分量设置每个驱动电机的动力
        leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
        rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
        leftBackDrive.setPower((rotY - rotX + rx) / denominator);
        rightBackDrive.setPower((rotY + rotX - rx) / denominator);
    }

    /**
     * @method handleLeftBumper()
     * 处理左肩键按钮按下的逻辑。
     * 第一次按下时，它会记录当前机器人姿态作为 'recordedPoseBasket'。
     * 随后的按下，它会命令机器人使用 Road Runner 自动驾驶到记录的 'recordedPoseBasket'。
     * 此外，它还控制竖爪舵机，并根据舵机的位置启动滑轨运动。
     */
    private void handleLeftBumper() {
        if (isLeftBumperFirstPress) { // 第一次按下逻辑：记录姿态
            recordedPoseBasket = drive.pose; // 记录当前机器人姿态
            telemetry.addData("Recorded Pose Basket", "X: %.2f, Y: %.2f, Heading: %.2f", recordedPoseBasket.position.x, recordedPoseBasket.position.y, Math.toDegrees(recordedPoseBasket.heading.toDouble())); // 遥测显示记录的姿态
            isLeftBumperFirstPress = false; // 第一次按下后设置标志为 false
        } else {  // 随后的按下逻辑：自动驾驶到记录的姿态
            Actions.runBlocking(drive.actionBuilder(drive.pose). // 使用 Road Runner 动作构建器创建并执行驱动动作
                    splineToSplineHeading(recordedPoseBasket, 0) // 创建样条曲线轨迹到记录的姿态，切线角度为 0
                    .build()); // 构建动作
            telemetry.addData("Current Pose Basket", "X: %.2f, Y: %.2f, Heading: %.2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble())); // 遥测显示自动驾驶后的当前姿态
        }
        telemetry.update(); // 更新遥测

        // 基于竖爪舵机位置的爪子和滑轨控制逻辑
        if (claw_shu.getPosition() == clawShuInitPosition) { // 如果竖爪舵机在初始位置
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION); // 旋转竖爪舵机
            isClawShuRotating = true; // 设置标志，指示竖爪正在旋转
        } else { // 如果竖爪舵机不在初始位置
            setSlidePosition(SLIDE_HIGH); // 将滑轨目标位置设置为 HIGH
            slideState = SlideState.MOVING_TO_POSITION; // 将滑轨状态更改为 MOVING_TO_POSITION
        }
    }

    /**
     * @method handleFrame()
     * 自动控制框架舵机。
     * 当 isFrameMoving 为 true 时，在延迟后将框架舵机移动到 HOLD 位置。
     * 否则，缓慢地将框架舵机移回 INITIAL 位置。
     */
    private void handleFrame() {
        if (isFrameMoving && frameCurrentPosition == FRAME_HOLD_POSITION && (System.currentTimeMillis() - frameMoveStartTime >= 490)) { // 检查框架是否正在移动，是否到达 HOLD 位置，以及延迟是否已过
            isFrameMoving = false; // 停止自动框架移动
        }
        if (!isFrameMoving) { // 如果框架没有自动移动
            frameCurrentPosition = Math.max(FRAME_INITIAL_POSITION, frameCurrentPosition - FRAME_SERVO_SPEED); // 逐渐将框架舵机移向 INITIAL 位置
            frame.setPosition(frameCurrentPosition); // 设置框架舵机位置
        }
    }

    /**
     * @method checkReset()
     * 检查是否按下 gamepad1 上的 'options' 按钮触发复位命令。
     * 如果按下按钮并防抖后，调用 resetAll() 方法。
     */
    private void checkReset() {
        if (gamepad1.options && debounce(lastOptionButtonPressTime)) { // 检查选项按钮是否按下并防抖
            lastOptionButtonPressTime = System.currentTimeMillis(); // 更新上次按下时间
            resetAll(); // 调用方法重置所有机器人系统
        }
    }

    /**
     * @method resetAll()
     * 将所有机器人机构和状态复位到其初始配置。
     * 包括舵机位置、爪子状态、滑轨状态和 IMU。
     */
    private void resetAll() {
        setInitialServoPositions(); // 将舵机设置到初始位置
        isClawOpen = false; // 关闭后爪
        clawPosition = 0.0; // 将后爪舵机位置设置为关闭
        backgrap.setPosition(clawPosition); // 应用后爪舵机位置
        isForwardClawOpen = false; // 关闭前爪
        forward_claw.setPosition(1); // 将前爪舵机位置设置为关闭
        isClawHengOpen = true; // 打开横爪
        claw_heng.setPosition(0.55); // 将横爪舵机位置设置为打开
        armForwardPosition = true; // 将前臂设置为初始位置
        arm_forward.setPosition(0.8); // 将前臂舵机位置设置为初始位置
        frameCurrentPosition = FRAME_INITIAL_POSITION; // 将框架舵机位置设置为初始位置
        frame.setPosition(FRAME_INITIAL_POSITION); // 应用框架舵机位置
        slideState = SlideState.IDLE; // 将滑轨状态设置为 IDLE
        resetIMU(); // 重置 IMU 和里程计
        clawShuCurrentPos = CLAW_SHU_INITIAL_POSITION; // 将竖爪舵机位置设置为初始位置
        claw_shu.setPosition(clawShuCurrentPos); // 应用竖爪舵机位置
        clawHengCurrentPos = 0.55; // 将横爪舵机位置设置为初始打开位置
        claw_heng.setPosition(clawHengCurrentPos); // 应用横爪舵机位置
        isLeftBumperFirstPress = true; // 复位左肩键首次按下标志
    }

    /**
     * @method initializeHardware()
     * 初始化所有用于驱动和挂钩的 DC 电机。
     * 从硬件图中获取电机实例，设置方向，并设置零动力行为。
     */
    private void initializeHardware() {
        // 从硬件图中获取 DC 电机实例，使用机器人配置文件中定义的名称
                leftFrontDrive = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        Left_Hanging_Motor = hardwareMap.get(DcMotor.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotor.class, "RightHangingMotor");

        // 基于机器人布线配置设置电机方向
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // 将挂钩电机的零动力行为设置为 BRAKE (制动)，以便在动力为零时保持位置
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 复位挂钩电机的编码器计数
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 设置挂钩电机使用编码器运行，用于位置控制
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @method initializeServos()
     * 初始化所有用于控制机器人机构的舵机对象。
     * 从硬件图中获取舵机实例，并为一些舵机设置初始位置。
     */
    private void initializeServos() {
        // 从硬件图中获取舵机实例，使用机器人配置文件中定义的名称
        backgrap = hardwareMap.get(Servo.class, "backgrap");
        forward_slide = hardwareMap.get(Servo.class, "forward_slide");
        arm_forward = hardwareMap.get(Servo.class, "arm_forward");
        claw_shu = hardwareMap.get(Servo.class, "claw_shu");
        forward_claw = hardwareMap.get(Servo.class, "forward_claw");
        claw_heng = hardwareMap.get(Servo.class, "claw_heng");
        frame = hardwareMap.get(Servo.class, "frame");

        // 为一些舵机设置初始位置
        frame.setPosition(FRAME_INITIAL_POSITION);
        backgrap.setPosition(clawPosition);
        claw_heng.setPosition(0.55);
        forward_claw.setPosition(1);
    }

    /**
     * @method initializeOdometry()
     * 初始化 GoBilda Pinpoint 里程计系统。
     * 从硬件图中获取里程计驱动器实例，设置偏移量、编码器分辨率和方向。
     * 还会重置 IMU 并等待一小段时间。
     */
    private void initializeOdometry() {
        // 获取 GoBilda Pinpoint 驱动器实例
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "goBILDAPinpoint");
        odo.setOffsets(5.5, 121.0); // 设置轮距和轴距偏移量 (根据你的机器人调整这些值)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); // 基于里程计舱类型设置编码器分辨率
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED); // 基于里程计舱安装设置编码器方向
        resetIMU(); // 在初始化时重置 IMU 和里程计
        sleep(200); // 在 IMU 重置后等待 200 毫秒
    }

    /**
     * @method setInitialServoPositions()
     * 为需要在程序开始时处于特定状态的舵机设置初始位置。
     */
    private void setInitialServoPositions() {
        forward_slide.setPosition(1); // 收回前滑轨
        arm_forward.setPosition(0.8); // 抬起前臂
        claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION); // 将竖爪舵机设置到初始位置
    }

    /**
     * @method controlClaw()
     * 使用 gamepad1 上的正方形按钮控制后爪 (backgrap)。
     * 每次按下按钮时，切换爪子的打开/关闭状态，并相应地调整舵机位置。
     */
    private void controlClaw() {
        if (gamepad1.square && debounce(lastSquareButtonPressTime)) { // 检查正方形按钮是否按下并防抖
            lastSquareButtonPressTime = System.currentTimeMillis(); // 更新上次按下时间
            clawPosition = isClawOpen ? Math.max(0.0, clawPosition - CLAW_INCREMENT) : Math.min(1.0, clawPosition + CLAW_INCREMENT); // 基于当前状态增量/减量爪子位置
            backgrap.setPosition(clawPosition); // 设置后爪舵机位置
            isClawOpen = !isClawOpen; // 切换爪子的打开/关闭状态
        }
    }

    /**
     * @method controlServos()
     * 基于 gamepad1 输入控制各种舵机 (前滑轨、前臂、前爪、横爪)。
     * 使用 D-pad 按钮、圆形、交叉和三角形按钮进行舵机控制。
     */
    private void controlServos() {
        // D-pad 上: 将前滑轨和手臂移动到交付位置，旋转竖爪
        if (gamepad1.dpad_up && debounce(lastForwardButtonPressTime)) {
            lastForwardButtonPressTime = System.currentTimeMillis();
            forward_slide.setPosition(0.4); // 部分伸出前滑轨
            arm_forward.setPosition(0.4); // 部分降低前臂
            claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION); // 旋转竖爪
        }
        // D-pad 下: 收回前滑轨和手臂，关闭前爪，复位竖爪和横爪
        if (gamepad1.dpad_down && debounce(lastBackButtonPressTime)) {
            lastBackButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(0); // 关闭前爪
            sleep(250); // 等待爪子关闭后再收回滑轨
            forward_slide.setPosition(1); // 完全收回前滑轨
            arm_forward.setPosition(0.8); // 完全抬起前臂
            claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION); // 将竖爪复位到初始位置
            claw_heng.setPosition(0.55); // 将横爪复位到初始打开位置
        }

        // 圆形按钮: 切换前爪打开/关闭
        if (gamepad1.circle && debounce(lastCircleButtonPressTime)) {
            lastCircleButtonPressTime = System.currentTimeMillis();
            forward_claw.setPosition(isForwardClawOpen ? 0.5 : 1); // 切换前爪舵机位置
            isForwardClawOpen = !isForwardClawOpen; // 切换前爪打开/关闭状态
        }
        // 交叉按钮: 在两个预设位置之间切换前臂位置
        if (gamepad1.cross && debounce(lastCrossButtonPressTime)) {
            lastCrossButtonPressTime = System.currentTimeMillis();
            if(armForwardPosition) { // 如果手臂在初始位置
                arm_forward.setPosition(0.15); // 将手臂移动到另一个较低的位置
                armForwardPosition = false; // 更新手臂位置状态
            } else { // 如果手臂在较低的位置
                arm_forward.setPosition(0.4); // 将手臂移回初始位置
                armForwardPosition = true; // 更新手臂位置状态
            }
        }

        // 三角形按钮: 切换横爪打开/关闭
        if (gamepad1.triangle && debounce(lastTriangleButtonPressTime)) {
            lastTriangleButtonPressTime = System.currentTimeMillis();
            claw_heng.setPosition(isClawHengOpen ? 0.07 : 0.55); // 切换横爪舵机位置
            isClawHengOpen = !isClawHengOpen; // 切换横爪打开/关闭状态
        }
    }

    /**
     * @method setSlidePosition(int position)
     * 设置线性滑轨电机的目标位置并开始将它们移动到该位置。
     * 使用 RUN_TO_POSITION 模式并将电机动力设置为 1。
     * @param position - 滑轨电机的目标编码器位置。
     */
    private void setSlidePosition(int position) {
        // 将零动力行为设置为 FLOAT (浮动)，以便在使用 RUN_TO_POSITION 时电机可以自由移动
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 设置两个挂钩电机的目标位置
        Left_Hanging_Motor.setTargetPosition(position);
        Right_Hanging_Motor.setTargetPosition(position);

        // 如果电机模式不是 RUN_TO_POSITION，则设置为 RUN_TO_POSITION 模式
        if (Left_Hanging_Motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // 将电机动力设置为 1，开始移动到目标位置
        Left_Hanging_Motor.setPower(1);
        Right_Hanging_Motor.setPower(1);
    }

    /**
     * @method isSlideBusy()
     * 检查滑轨电机是否仍在移动到其目标位置 (忙碌)。
     * @return boolean - 如果左电机或右电机仍处于忙碌状态 (未到达目标)，则返回 true。
     */
    private boolean isSlideBusy() {
        return !Left_Hanging_Motor.isBusy() && !Right_Hanging_Motor.isBusy();
    }

    /**
     * @method handleSlideMovement()
     * 状态机，用于基于手柄输入和当前状态控制线性滑轨机构。
     * 处理不同的滑轨状态，如 IDLE (空闲)、MOVING_TO_POSITION (移动到位置)、MANUAL_DOWN (手动下降) 和 HANGING_PAUSE (挂钩暂停)。
     */
    private void handleSlideMovement() {
        // 状态: IDLE - 滑轨未主动移动，电机处于制动状态
        if (slideState == SlideState.IDLE) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 设置电机为制动模式
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Y 按钮 (遥测消息 - 无动作)
            if (gamepad1.y) {
                telemetry.addData("滑轨状态", "已复位到零点");
            }
            // 从 IDLE 状态基于手柄输入的状态转换
            else if (slideState == SlideState.IDLE) {
                // 左肩键: 移动到 HIGH 位置 (如果姿态已记录，则触发自动驾驶)
                if (gamepad1.left_bumper) {
                    handleLeftBumper(); // 调用 handleLeftBumper 方法处理左肩键逻辑
                    isAutoSlideDown = true; // 设置标志，指示到达高位后自动滑轨下降
                }
                // D-pad 左: 移动到 MID 位置
                else if (gamepad1.dpad_left) {
                    Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // 临时设置为浮动模式
                    Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    setSlidePosition(SLIDE_MID); // 将滑轨目标位置设置为 MID
                    slideState = SlideState.MOVING_TO_POSITION; // 将状态更改为 MOVING_TO_POSITION
                }
                // D-pad 右: 移动到 HANG 位置
                else if (gamepad1.dpad_right) {
                    Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // 临时设置为浮动模式
                    Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    setSlidePosition(SLIDE_HANG); // 将滑轨目标位置设置为 HANG
                    slideState = SlideState.MOVING_TO_POSITION; // 将状态更改为 MOVING_TO_POSITION
                }

                // 右肩键 (按住): 手动滑轨下降控制
                if (gamepad1.right_bumper) {
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // 切换到 RUN_WITHOUT_ENCODER 以进行手动控制
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Left_Hanging_Motor.setPower(SLIDE_DOWN_POWER); // 设置负动力以使滑轨下降
                    Right_Hanging_Motor.setPower(SLIDE_DOWN_POWER);
                    slideState = SlideState.MANUAL_DOWN; // 将状态更改为 MANUAL_DOWN
                }
            }

        }
        // 状态: MOVING_TO_POSITION - 滑轨正在移动到目标编码器位置
        else if (slideState == SlideState.MOVING_TO_POSITION && isSlideBusy() ) {
            // 基于到达目标位置和当前目标位置的状态转换逻辑
            if(Left_Hanging_Motor.getTargetPosition() == SLIDE_HANG){ // 如果目标是 HANG 位置
                hangStartTime = System.currentTimeMillis(); // 记录挂钩开始时间
                backgrap.setPosition(0); // 到达挂钩位置时关闭后爪
                slideState = SlideState.HANGING_PAUSE; // 将状态更改为 HANGING_PAUSE
            } else if (Left_Hanging_Motor.getTargetPosition() == 0){ // 如果目标是 HOME 位置 (0)
                slideState = SlideState.IDLE; // 将状态改回 IDLE
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 设置电机为制动模式
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0); // 停止电机
                Right_Hanging_Motor.setPower(0);
            }else if (Left_Hanging_Motor.getTargetPosition() == SLIDE_MID){ // 如果目标是 MID 位置
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 设置电机为制动模式
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideState = SlideState.IDLE; // 将状态改回 IDLE
                Left_Hanging_Motor.setPower(0); // 停止电机
                Right_Hanging_Motor.setPower(0);
            }

        }
        // 状态: HANGING_PAUSE - 到达 HANG 位置后的暂停状态，用于在收回前计时
        else if (slideState == SlideState.HANGING_PAUSE) {
            if (System.currentTimeMillis() - hangStartTime > 1000) { // 检查 1 秒的暂停持续时间是否已过
                setSlidePosition(0); // 将滑轨目标位置设置为 HOME (0) 以收回
                slideState = SlideState.MOVING_TO_POSITION; // 将状态更改为 MOVING_TO_POSITION 以收回
            }
        }
        // 状态: MANUAL_DOWN - 在按住右肩键时手动向下移动滑轨
        else if (slideState == SlideState.MANUAL_DOWN) {
            if (!gamepad1.right_bumper) { // 检查右肩键是否释放
                Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 释放肩键时设置为制动模式
                Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Left_Hanging_Motor.setPower(0); // 停止电机
                Right_Hanging_Motor.setPower(0);
                slideState = SlideState.IDLE; // 将状态改回 IDLE
            }
        }
        // 状态: 后备 - 如果滑轨忙碌但状态不是 MANUAL_DOWN、MOVING_TO_POSITION 或 HANGING_PAUSE，则停止电机并设置为 IDLE
        else if (isSlideBusy()) {
            Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 设置为制动模式
            Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (slideState == SlideState.MOVING_TO_POSITION) { // 如果状态是 MOVING_TO_POSITION
                slideState = SlideState.IDLE; // 将状态更改为 IDLE
            }
            Left_Hanging_Motor.setPower(0); // 停止电机
            Right_Hanging_Motor.setPower(0);
        }
        // 竖爪舵机旋转逻辑
        if (isClawShuRotating) {
            if (claw_shu.getPosition() == CLAW_SHU_ROTATE_POSITION) { // 检查竖爪舵机是否到达旋转位置
                sleep(300); // 等待 300 毫秒
                setSlidePosition(SLIDE_HIGH); // 将滑轨目标位置设置为 HIGH
                slideState = SlideState.MOVING_TO_POSITION; // 将状态更改为 MOVING_TO_POSITION
                isClawShuRotating = false; // 复位竖爪旋转标志
            }
        }
        // 框架舵机自动移动逻辑 (当滑轨到达 SLIDE_HIGH_START 高度时触发)
        if (slideState == SlideState.MOVING_TO_POSITION && Left_Hanging_Motor.getCurrentPosition() >= SLIDE_HIGH_START && !isFrameMoving) {
            frameMoveStartTime = System.currentTimeMillis(); // 记录框架移动开始时间
            frameCurrentPosition = FRAME_HOLD_POSITION; // 将框架舵机目标位置设置为 HOLD
            frame.setPosition(frameCurrentPosition); // 将框架舵机移动到 HOLD 位置
            isFrameMoving = true; // 设置框架移动标志
        }
        // 自动滑轨下降逻辑 (在左肩键序列和滑轨到达 IDLE 状态后触发)
        if (isAutoSlideDown && slideState == SlideState.IDLE && !isFrameMoving) {
            setSlidePosition(SLIDE_HOME); // 将滑轨目标位置设置为 HOME (0) 以进行自动滑轨下降
            slideState = SlideState.MOVING_TO_POSITION; // 将状态更改为 MOVING_TO_POSITION 以进行滑轨下降
            isAutoSlideDown = false; // 复位自动滑轨下降标志
        }
    }

    /**
     * @method debounce(long lastPressTime)
     * 对按钮按下进行防抖处理，防止单次按钮按下触发多次。
     * @param lastPressTime - 上次按钮按下的时间戳。
     * @return boolean - 如果自上次按下时间以来防抖延迟已过，则返回 true，允许新的触发。
     */
    private boolean debounce(long lastPressTime) {
        return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY; // 检查自上次按下时间是否大于防抖延迟
    }

    /**
     * @method updateTelemetry(double robotHeading)
     * 更新遥测数据，在 FTC 驱动站应用程序上显示。
     * 显示机器人状态、IMU 航向角、滑轨电机位置、舵机位置、爪子状态、框架状态和控制提示。
     * @param robotHeading - 要在遥测中显示的当前机器人航向角，单位为度。
     */
    private void updateTelemetry(double robotHeading) {
        telemetry.addData("状态", "运行中"); // 显示程序状态为 "运行中"
        telemetry.addData("GoBildaIMU", "%.2f", robotHeading); // 显示 IMU 航向角
        telemetry.addData("滑轨", "左:%d, 右:%d", Left_Hanging_Motor.getCurrentPosition(), Right_Hanging_Motor.getCurrentPosition()); // 显示滑轨电机编码器位置
        telemetry.addData("前段", "滑轨:%.2f, 小臂:%.2f, 竖:%.2f, 横:%.2f, 前爪:%.2f", forward_slide.getPosition(), arm_forward.getPosition(), claw_shu.getPosition(), claw_heng.getPosition(), forward_claw.getPosition()); // 显示前部机构舵机位置
        telemetry.addData("后爪", "状态: %s, 位置:%.2f", isClawOpen ? "打开" : "关闭", clawPosition); // 显示后爪状态和位置
        telemetry.addData("框状态", "位置: %.2f", frame.getPosition()); // 显示框架舵机位置
        telemetry.addData("框舵机目标位置:", frameCurrentPosition); // 显示框架舵机目标位置
        telemetry.addData("控制", "左肩=高位, D-pad左=中位, D-pad右=挂钩, 长按右肩=下降, Y=复位,右摇杆按钮=IMU重置"); // 显示驾驶员控制提示
        telemetry.addData("提示", "按下圆形按键控制前爪, 右摇杆控制前爪自由度,  三角键：前爪横向自由度， 交叉键：小臂"); // 显示附加控制提示
        telemetry.update(); // 更新遥测，将数据发送到驱动站
    }
}