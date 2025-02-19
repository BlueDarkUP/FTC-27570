package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.GoBildaPinpointDriver;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "比赛用手动程序+视觉抓取", group = "Competition")
public class SingleStickWithArmWithVision extends LinearOpMode {
    private MecanumDrive drive;
    private static final double CLAW_INCREMENT = 0.55;
    private static final double DEBOUNCE_DELAY = 300;
    private static final double BACK_ARM_RESET_POSITION = 0.07;
    private static final int LIFT_UP_POSITION = 610;
    private static final double BACK_ARM_SET_POSITION = 0.82;
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

    public boolean isDpadLeftButtonPreviousState() {
        return dpadLeftButtonPreviousState;
    }

    public void setDpadLeftButtonPreviousState(boolean dpadLeftButtonPreviousState) {
        this.dpadLeftButtonPreviousState = dpadLeftButtonPreviousState;
    }

    private enum SlideState {IDLE, MOVING_TO_POSITION, MANUAL_DOWN, HANGING_PAUSE}

    private SlideState slideState = SlideState.IDLE;
    private Servo backArmServo;
    private Servo backgrapServo;
    private DcMotor bigArmMotor;

    private boolean isClawShuRotating = false, isFrameMoving = false, isAutoSlideDown = false, isClawOpen = false, isForwardClawOpen = false, isClawHengOpen = true, armForwardPosition = true, isFirstReset = true, isLeftBumperFirstPress = true;
    private double clawPosition = 0.0, clawShuCurrentPos = 0.0, clawHengCurrentPos = 0.0, frameCurrentPosition = FRAME_INITIAL_POSITION, clawShuInitPosition = 0.0, initialHeading = 0;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Left_Hanging_Motor, Right_Hanging_Motor;
    private Servo backgrap, forward_slide, arm_forward, claw_shu, forward_claw, claw_heng, frame, forward_slide_2;
    private GoBildaPinpointDriver odo;

    private long lastOptionButtonPressTime = 0;
    private long lastSquareButtonPressTime = 0; // Changed to square button for vision
    private long lastRightStickPressTime = 0;
    private long frameMoveStartTime = 0;
    private long lastDpadRightPressTime = 0;
    private long lastDpadLeftPressTime = 0;
    private long lastDpadUpPressTime = 0;
    private long lastLeftBumperPressTime = 0;
    private long lastRightBumperPressTime = 0;
    private long lastCirclePressTime = 0; // For vision trigger

    private final double defaultServoPosition = 0.08;
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

    // Vision related variables and constants (Copied from VisionMovementOutput)
    private enum CameraType {
        INTERNAL_CAMERA_V1,
        INTERNAL_CAMERA_V2,
        WEBCAM
    }
    private static final CameraType CAMERA_TYPE = CameraType.WEBCAM;
    private static final double PIXELS_TO_CM_RATIO = 0.021875;
    private static final double CLAW_OFFSET_FROM_CAMERA_CM = 0;
    private static final String ALLIANCE_COLOR = "red";
    private static final int MIN_REGION_WIDTH = 10;
    private static final int MIN_REGION_HEIGHT = 10;
    private static final double CLAW_CENTER_X_CM = -0.5;
    private static final double CLAW_CENTER_Y_CM = 4.64;
    private static final double SERVO_CENTER_POSITION_HENG = ServoPositionsVision.CLAW_HENG_DEFAULT;
    private static final double ANGLE_OFFSET_HENG = 0;
    private static final double DIRECTION_MULTIPLIER_HENG = 1.0;
    private static final double CLAW_HORIZONTAL_ERROR_CM = -5.7;
    private static final double SERVO_ANGLE_COEFFICIENT = -0.006;
    private static final double SINGLE_SIDE_MAX_SIZE_CM = 404.5714285714286;
    private static final double DOUBLE_SIDE_MAX_SIZE_CM = 405.6772618921679;
    private static final int SINGLE_SIDE_MAX_SIZE = (int) (SINGLE_SIDE_MAX_SIZE_CM / PIXELS_TO_CM_RATIO);
    private static final int DOUBLE_SIDE_MAX_SIZE = (int) (DOUBLE_SIDE_MAX_SIZE_CM / PIXELS_TO_CM_RATIO);
    private static final double MIN_AREA_THRESHOLD_PIXELS = 102645.5510204082;
    private static final Scalar RED_LOWER_1 = new Scalar(170, 100, 100);
    private static final Scalar RED_UPPER_1 = new Scalar(180, 255, 255);
    private static final Scalar RED_LOWER_2 = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER_2 = new Scalar(10, 255, 255);
    private static final Scalar BLUE_LOWER = new Scalar(100, 125, 220);
    private static final Scalar BLUE_UPPER = new Scalar(130, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(20, 145, 250);
    private static final Scalar YELLOW_UPPER = new Scalar(40, 255, 255);

    private boolean debounce(long lastPressTime) {
        return (System.currentTimeMillis() - lastPressTime) > DEBOUNCE_DELAY;
    }

    private static class ServoPositionsVision { // Renamed to avoid conflict
        static final double ARM_FORWARD_DEFAULT = 0.65;
        static final double CLAW_SHU_DEFAULT = 0.32;
        static final double CLAW_HENG_DEFAULT = 0.54;
        static final double FORWARD_SLIDE_DEFAULT = 0.53;
        static final double ARM_FORWARD_OVERRIDE = 0.4;
        static final double CLAW_SHU_OVERRIDE = 1.0;
        static final double FORWARD_SLIDE_OVERRIDE = 0.53;
        static final double FORWARD_CLAW_DEFAULT = 0.85;
        static final double FORWARD_CLAW_GRAB = 0.0;
        public static double reversepose = 0.57;
    }

    private OpenCvCamera camera;
    private WebcamName webcamName; // Declare webcamName
    private ColorDetectionPipelineImpl colorDetectionPipeline;
    private final int cameraWidth = 1280;
    private final int cameraHeight = 720;
    private volatile boolean isCameraInitialized = false;
    private boolean processFrameFlag = false;
    private boolean isApproached = false;
    private boolean isStreaming = false;


    @Override
    public void runOpMode() {
        initializeHardware();
        initializeServos();
        initializeOdometry();
        setInitialServoPositions();
        clawShuInitPosition = claw_shu.getPosition();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        if (!initializeCamera()) { // Initialize camera here
            telemetry.addLine("摄像头初始化失败，请检查!");
            telemetry.update();
        } else {
            telemetry.addLine("摄像头初始化成功!");
            telemetry.update();
        }


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

            if (gamepad1.circle && debounce(lastCirclePressTime)) { // Trigger vision with circle button
                lastCirclePressTime = System.currentTimeMillis();
                processFrameAndControlServos();
            }

            updateTelemetry(robotHeading);
        }
        stopCamera(); // Stop camera when OpMode ends
    }

    private boolean initializeCamera() {
        telemetry.addLine("初始化摄像头...");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        try {
            switch (CAMERA_TYPE) {
                case INTERNAL_CAMERA_V1:
                    camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                    break;
                case INTERNAL_CAMERA_V2:
                    camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
                    break;
                case WEBCAM:
                    webcamName = hardwareMap.get(WebcamName.class, "Webcam"); // Get WebcamName from hardwareMap
                    if (webcamName == null) {
                        telemetry.addData("摄像头错误", "找不到名为 'Webcam' 的摄像头硬件配置.");
                        telemetry.update();
                        return false;
                    }
                    camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
                    break;
                default:
                    telemetry.addData("摄像头错误", "未知的摄像头类型.");
                    telemetry.update();
                    return false;
            }

            if (camera == null) {
                telemetry.addData("摄像头错误", "创建摄像头实例失败 (camera == null).");
                telemetry.update();
                return false;
            }

            colorDetectionPipeline = new ColorDetectionPipelineImpl();
            camera.setPipeline(colorDetectionPipeline);

            telemetry.addLine("尝试打开摄像头设备异步...");
            telemetry.update();

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    telemetry.addLine("摄像头设备已打开 (onOpened 回调).");
                    telemetry.update();
                    isCameraInitialized = true;
                    telemetry.addData("摄像头状态", "已打开，但推流未开始");
                    telemetry.update();
                    isStreaming = false;
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("摄像头错误", "打开摄像头设备失败 (onError 回调).");
                    telemetry.addData("错误代码", errorCode);
                    telemetry.update();
                    isCameraInitialized = true;
                }
            });

            telemetry.addLine("等待摄像头初始化完成...");
            telemetry.update();
            while (!isStarted() && !isCameraInitialized) {
                sleep(100);
            }

            if (!isCameraInitialized) {
                telemetry.addData("摄像头状态", "初始化超时或失败, OpMode 已停止.");
                telemetry.update();
                return false;
            }
            telemetry.addLine("摄像头初始化完成，推流尚未开始");
            telemetry.update();
            return true;

        } catch (Exception e) {
            telemetry.addData("摄像头错误", "初始化异常: " + e.getMessage());
            telemetry.update();
            return false;
        }
    }

    private void stopCamera() {
        if (camera != null && isStreaming) {
            camera.stopStreaming();
            camera.closeCameraDevice();
            telemetry.addLine("摄像头流已停止且设备已关闭");
            telemetry.update();
            isStreaming = false;
        } else if (camera != null && !isStreaming) {
            camera.closeCameraDevice();
            telemetry.addLine("摄像头设备已关闭 (推流未运行)");
            telemetry.update();
        }
        camera = null;
    }

    private void processFrameAndControlServos() {
        if (!processFrameFlag) {
            processFrameFlag = true;

            if (!isStreaming) {
                telemetry.addLine("启动摄像头推流...");
                telemetry.update();
                camera.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                isStreaming = true;
                sleep(200);
                telemetry.addLine("摄像头推流已启动.");
                telemetry.update();
            }

            List<DetectedCube> cubes = colorDetectionPipeline.getDetectedCubes();
            double moveForwardCm = colorDetectionPipeline.getMoveForward();
            double moveSidewaysCm = colorDetectionPipeline.getMoveSideways();
            double servoPositionOffset = colorDetectionPipeline.getServoPositionOffset();

            telemetry.addLine("运动数据 (按下按钮时):");
            telemetry.addData("前进 (cm)", String.format(Locale.US, "%.2f", moveForwardCm));
            telemetry.addData("侧向 (cm)", String.format(Locale.US, "%.2f", moveSidewaysCm));
            telemetry.addData("舵机偏移量", String.format(Locale.US, "%.3f", servoPositionOffset));

            DetectedCube closestCube = colorDetectionPipeline.getClosestCube();

            if (!cubes.isEmpty() && closestCube != null && closestCube.boundingBox.area() < MIN_AREA_THRESHOLD_PIXELS && !isApproached) {
                isApproached = true;
                telemetry.addLine("检测到小方块，执行接近动作...");
                telemetry.update();

                try {
                    Vector2d targetPosition = new Vector2d(
                            drive.pose.position.x + closestCube.centerXCm * 0.39370,
                            drive.pose.position.y - closestCube.centerYCm * 0.39370
                    );
                    Action approachMovement = drive.actionBuilder(drive.pose)
                            .splineToConstantHeading(targetPosition, 0)
                            .build();
                    Actions.runBlocking(approachMovement);
                    telemetry.addLine("Road Runner 接近动作完成.");
                } catch (Exception e) {
                    telemetry.addLine("*** Road Runner 接近动作报错 ***");
                    telemetry.addLine("异常信息: " + e.getMessage());
                }
            } else {
                if (isApproached) {
                    telemetry.addLine("已完成接近动作，现在执行抓取...");
                    isApproached = false;
                } else if (!cubes.isEmpty()) {
                    telemetry.addLine("检测到足够大的方块，执行抓取动作...");
                } else {
                    telemetry.addLine("未检测到方块，不执行动作。");
                    isApproached = false;
                    processFrameFlag = false;
                    telemetry.update();
                    return;
                }
                telemetry.update();


                if (!cubes.isEmpty()) {
                    try {
                        Action visionBasedMovement = drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(
                                        new Vector2d(drive.pose.position.x + moveForwardCm * 0.39370, drive.pose.position.y - moveSidewaysCm * 0.39370), 0
                                )
                                .build();

                        Actions.runBlocking(visionBasedMovement);
                        telemetry.addLine("Road Runner 抓取动作完成.");
                    } catch (Exception e) {
                        telemetry.addLine("*** Road Runner 抓取动作报错 ***");
                        telemetry.addLine("异常信息: " + e.getMessage());
                    }

                    double targetHengServoPosition = SERVO_CENTER_POSITION_HENG + servoPositionOffset;
                    targetHengServoPosition = Range.clip(targetHengServoPosition, 0, 1);

                    arm_forward.setPosition(ServoPositionsVision.ARM_FORWARD_OVERRIDE); // arm_forward
                    claw_heng.setPosition(targetHengServoPosition);
                    double reversepos = 0.57;
                    forward_slide.setPosition(ServoPositionsVision.reversepose);
                    forward_slide_2.setPosition(ServoPositionsVision.FORWARD_SLIDE_OVERRIDE);// forward_slide
                    claw_shu.setPosition(1);

                    sleep(300);
                    arm_forward.setPosition(0.15); // arm_forward
                    sleep(300);
                    forward_claw.setPosition(ServoPositionsVision.FORWARD_CLAW_GRAB);
                    boolean isClawDown = true;
                    sleep(300);
                    arm_forward.setPosition(0.4); // arm_forward
                    sleep(1000);
                    arm_forward.setPosition(ServoPositionsVision.ARM_FORWARD_DEFAULT); // arm_forward
                    claw_shu.setPosition(ServoPositionsVision.CLAW_SHU_DEFAULT);
                    claw_heng.setPosition(ServoPositionsVision.CLAW_HENG_DEFAULT);
                    forward_slide.setPosition(ServoPositionsVision.reversepose); // forward_slide
                    forward_slide_2.setPosition(ServoPositionsVision.FORWARD_SLIDE_DEFAULT);
                    forward_claw.setPosition(ServoPositionsVision.FORWARD_CLAW_DEFAULT);
                }
            }


            telemetry.update();
            while (gamepad1.circle && opModeIsActive()) {
                sleep(50);
            }
            processFrameFlag = false;
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

        leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
        rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
        leftBackDrive.setPower((rotY - rotX + rx) / denominator);
        rightBackDrive.setPower((rotY + rotX - rx) / denominator);
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
        armHangingState = false;
        dpadLeftButtonPreviousState = false;
        isForwardSlideExtended = false;
        isDpadLeftUsed = false;
        isDpadRightUsed = false;
        recordedCatchPose = null;
        recordedChamberPose = null;
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
        IMU imu = hardwareMap.get(IMU.class, "imu");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam"); // Initialize webcamName

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
        forward_slide.setDirection(Servo.Direction.REVERSE);

        frame.setPosition(FRAME_INITIAL_POSITION);
        backgrap.setPosition(clawPosition);
        claw_heng.setPosition(0.55);
        forward_claw.setPosition(1);
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
        if (gamepad1.square && debounce(lastSquareButtonPressTime)) { // Still using square for manual back claw
            lastSquareButtonPressTime = System.currentTimeMillis();
            clawPosition = isClawOpen ? Math.max(0.0, clawPosition - CLAW_INCREMENT) : Math.min(1.0, clawPosition + CLAW_INCREMENT);
            backgrap.setPosition(clawPosition);
            isClawOpen = !isClawOpen;
        }
    }

    private void controlServos() {
        if (gamepad1.right_bumper && debounce(lastRightBumperPressTime)) {
            lastRightBumperPressTime = System.currentTimeMillis();
            if (!isForwardSlideExtended) {
                forward_slide.setPosition(0.53);
                forward_slide_2.setPosition(0.53);
                arm_forward.setPosition(0.4);
                claw_shu.setPosition(CLAW_SHU_ROTATE_POSITION);
                telemetry.addLine("--- Right Bumper Pressed - Extend ---");
                telemetry.addData("Arm Forward Position (Right Bumper Extend)", arm_forward.getPosition());
                telemetry.update();
                isForwardSlideExtended = true;
            } else {
                forward_claw.setPosition(0);
                sleep(250);
                forward_slide.setPosition(0);
                forward_slide_2.setPosition(0);
                arm_forward.setPosition(0.8);
                claw_shu.setPosition(CLAW_SHU_INITIAL_POSITION);
                claw_heng.setPosition(0.55);
                telemetry.addLine("--- Right Bumper Pressed - Retract ---");
                telemetry.addData("Arm Forward Position (Right Bumper Retract)", arm_forward.getPosition());
                telemetry.update();
                isForwardSlideExtended = false;
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
            if (gamepad1.left_bumper && debounce(lastLeftBumperPressTime)) {
                lastLeftBumperPressTime = System.currentTimeMillis();
                armHangingState = !armHangingState;

                setMotorBrakeMode(false);
                arm_forward.setPosition(1);

                double hangingMotorPower = 1.0;
                if (armHangingState) {
                    Left_Hanging_Motor.setTargetPosition(targetHangingMotorPosition);
                    Right_Hanging_Motor.setTargetPosition(targetHangingMotorPosition);
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Hanging_Motor.setPower(hangingMotorPower);
                    Right_Hanging_Motor.setPower(hangingMotorPower);
                    sleep(250);
                    bigArmMotor.setTargetPosition(targetMotorPosition);
                    double targetServoPosition = 0.84;
                    backArmServo.setPosition(targetServoPosition);

                    bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double targetMotorPower = 0.7;
                    bigArmMotor.setPower(targetMotorPower);


                } else {
                    backArmServo.setPosition(defaultServoPosition);
                    bigArmMotor.setTargetPosition(defaultMotorPosition);
                    Left_Hanging_Motor.setTargetPosition(defaultHangingMotorPosition);
                    Right_Hanging_Motor.setTargetPosition(defaultHangingMotorPosition);

                    bigArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    double defaultMotorPower = 0.35;
                    bigArmMotor.setPower(defaultMotorPower);
                    Left_Hanging_Motor.setPower(hangingMotorPower);
                    Right_Hanging_Motor.setPower(hangingMotorPower);

                    while (opModeIsActive() && (bigArmMotor.isBusy() || Left_Hanging_Motor.isBusy() || Right_Hanging_Motor.isBusy())) {
                        idle();
                    }
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


                drive.actionBuilder(drive.pose)
                        //初次抓快构建三次样条曲线之前复位执行机构
                        .stopAndAdd(new ChamberDrive5Park.ArmMotorAction((DcMotorEx) bigArmMotor, true, 0))
                        .stopAndAdd(new ChamberDrive5Park.ServoAction(backArmServo, BACK_ARM_RESET_POSITION))
                        .stopAndAdd(new ChamberDrive5Park.MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, 0))
                        .stopAndAdd(new ChamberDrive5Park.ServoAction(backgrap, 0))
                        .splineToSplineHeading(recordedCatchPose, 0)
                        .waitSeconds(0.1)
                        .stopAndAdd(new ChamberDrive5Park.ServoAction(backgrap, 0.6));

                int numberOfChamberPoints = 13;
                double yIncrement = 5 * 0.39370;

                for (int i = 0; i < numberOfChamberPoints; i++) {
                    Vector2d ChamberPoseInConstantHeading = new Vector2d(
                            current_chamber_pose.position.x - 0,
                            current_chamber_pose.position.y + 5 * 0.39370
                    );
                    drive.actionBuilder(drive.pose)
                            //在挂块之前伸出大臂，抬高滑轨，张开后爪
                            .stopAndAdd(new ChamberDrive5Park.MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, LIFT_UP_POSITION))
                            .waitSeconds(0.1)
                            .stopAndAdd(new ChamberDrive5Park.ArmMotorAction((DcMotorEx) bigArmMotor, false, targetMotorPosition))
                            .stopAndAdd(new ChamberDrive5Park.ServoAction(backArmServo, BACK_ARM_SET_POSITION))
                            .splineToConstantHeading(ChamberPoseInConstantHeading, 0)//挂杆位置
                            .waitSeconds(0.1)
                            .stopAndAdd(new ChamberDrive5Park.ServoAction(backgrap, 0))//打开后爪
                            .waitSeconds(0.1)
                            //在抓块之前复位
                            .stopAndAdd(new ChamberDrive5Park.ArmMotorAction((DcMotorEx) bigArmMotor, true, 0))
                            .stopAndAdd(new ChamberDrive5Park.ServoAction(backArmServo, BACK_ARM_RESET_POSITION))
                            .stopAndAdd(new ChamberDrive5Park.MotorAction((DcMotorEx) Left_Hanging_Motor, (DcMotorEx) Right_Hanging_Motor, 0))
                            .splineToConstantHeading(CatchPoseInConstantHeading, 0)
                            .waitSeconds(0.1)
                            .stopAndAdd(new ChamberDrive5Park.ServoAction(backgrap, 0.6))
                            .waitSeconds(0.1);


                    current_chamber_pose = new Pose2d(current_chamber_pose.position.x, current_chamber_pose.position.y + 5 * 0.39370 + yIncrement, current_chamber_pose.heading.toDouble());
                }

                Action AutoChamberDrive = drive.actionBuilder(drive.pose).build();
                Actions.runBlocking(AutoChamberDrive);


                telemetry.addData("操作", "Chamber Auto Drive 完成");
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

    @SuppressLint("DefaultLocale")
    private void updateTelemetry(double robotHeading) {
        telemetry.addData("状态", "运行中");
        telemetry.addData("GoBildaIMU", "%.2f", robotHeading);
        telemetry.addData("滑轨", "左:%d, 右:%d", Left_Hanging_Motor.getCurrentPosition(), Right_Hanging_Motor.getCurrentPosition());
        telemetry.addData("前段", "滑轨:%.2f, 小臂:%.2f, 竖:%.2f, 横:%.2f, 前爪:%.2f", forward_slide.getPosition(), arm_forward.getPosition(), claw_shu.getPosition(), claw_heng.getPosition(), forward_claw.getPosition());
        telemetry.addData("后爪", "状态: %s, 位置:%.2f", isClawOpen ? "打开" : "关闭", clawPosition);
        telemetry.addData("框状态", "位置: %.2f", frame.getPosition());
        telemetry.addData("框舵机目标位置:", frameCurrentPosition);
        telemetry.addData("控制", "左肩=臂架切换, 右摇杆按钮=IMU重置, D-pad左=设置catch pose, D-pad右=记录chamber pose (一次性), D-pad上= Chamber Auto Drive, 圆圈=视觉抓取"); // Added circle button info
        telemetry.addData("提示", "按下方形按键控制后爪, 右摇杆控制前爪自由度,  三角键：前爪横向自由度， 交叉键：小臂, 右肩键：前滑轨伸缩, 右扳机：滑轨下降");
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

        // Vision Telemetry (Merged from VisionMovementOutput's updateTelemetry)
        if (isStreaming && colorDetectionPipeline != null) {
            telemetry.addLine("--- 视觉数据 (按下圆圈按钮时更新) ---");
            telemetry.addData("帧计数", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("滑轨目标位置", "左:%d, 右:%d", Left_Hanging_Motor.getTargetPosition(), Right_Hanging_Motor.getTargetPosition());
            telemetry.addData("前爪", "状态: %s, 位置:%.2f", isForwardClawOpen ? "打开" : "关闭", forward_claw.getPosition());

            List<DetectedCube> currentCubes = colorDetectionPipeline.getDetectedCubes();
            if (!currentCubes.isEmpty()) {
                for (DetectedCube cube : currentCubes) {
                    telemetry.addLine("--- 检测到方块 ---");
                    telemetry.addData("颜色", cube.color);
                    telemetry.addData("图像坐标 (px)", String.format(Locale.US, "(%d, %d)", cube.centerXImagePx, cube.centerYImagePx));
                    telemetry.addData("世界坐标 (cm)", String.format(Locale.US, "(%.1f, %.1f)", cube.centerXCm, cube.centerYCm));
                    telemetry.addData("角度 (度)", String.format(Locale.US, "%.1f", cube.angleDegrees));
                    telemetry.addData("宽高比 (短/长)", String.format(Locale.US, "%.2f", cube.aspectRatio));
                    telemetry.addData("面积 (pixels^2)", String.format(Locale.US, "%.2f", cube.boundingBox.area()));
                }
            } else {
                telemetry.addLine("未检测到方块 (视觉)");
            }

            double moveForward = colorDetectionPipeline.getMoveForward();
            double moveSideways = colorDetectionPipeline.getMoveSideways();
            double servoPositionOffset = colorDetectionPipeline.getServoPositionOffset();

            if (!currentCubes.isEmpty()) {
                DetectedCube closestCube = colorDetectionPipeline.getClosestCube();
                if (closestCube != null) {
                    telemetry.addData("目标颜色", closestCube.color.toUpperCase());
                    telemetry.addData("前进距离 (cm)", String.format(Locale.US, "%.1f", moveForward));
                    telemetry.addData("横向移动 (cm)", String.format(Locale.US, "%.1f", moveSideways));
                    telemetry.addData("舵机偏移量", String.format(Locale.US, "%.3f", servoPositionOffset));
                    telemetry.addData("直线距离 (cm)", String.format(Locale.US, "%.1f", colorDetectionPipeline.getDistanceToClosestCube()));
                    telemetry.addData("目标面积 (pixels^2)", String.format(Locale.US, "%.2f", closestCube.boundingBox.area()));
                } else {
                    telemetry.addLine("未找到有效目标方块 (视觉)");
                }
            } else {
                telemetry.addLine("未检测到长方体，无运动指令 (视觉)");
            }
            telemetry.addData("claw_heng 目标位置", String.format("%.3f", (SERVO_CENTER_POSITION_HENG + colorDetectionPipeline.getServoPositionOffset())));
            telemetry.addData("claw_heng 当前位置", String.format("%.3f", claw_heng.getPosition()));

        } else if (!isStreaming) {
            telemetry.addLine("摄像头推流已停止 (视觉)");
        }


        telemetry.addData("Arm Forward Servo", "Position: %.2f", arm_forward.getPosition());
        telemetry.addData("armForwardPosition Boolean", armForwardPosition);
        telemetry.update();

    }

    private void setMotorBrakeMode(boolean brake) {
        DcMotor.ZeroPowerBehavior behavior = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        bigArmMotor.setZeroPowerBehavior(behavior);
        Left_Hanging_Motor.setZeroPowerBehavior(behavior);
        Right_Hanging_Motor.setZeroPowerBehavior(behavior);
        motorsInBrake = brake;
    }


    // --- ColorDetectionPipelineImpl and DetectedCube classes (Copied from VisionMovementOutput) ---
    class ColorDetectionPipelineImpl extends OpenCvPipeline {
        private final Mat hsvImage = new Mat();
        private final Mat mask = new Mat();
        private final List<DetectedCube> detectedCubes = new ArrayList<>();
        private boolean viewportPaused;

        private final Mat redMask1 = new Mat();
        private final Mat redMask2 = new Mat();
        private final Mat blueMask = new Mat();
        private final Mat yellowMask = new Mat();

        private static final int MOVING_AVERAGE_WINDOW_SIZE = 5;
        private final List<Double> moveForwardHistory = new ArrayList<>();
        private final List<Double> moveSidewaysHistory = new ArrayList<>();


        private int clawCenterXPixel;
        private int clawCenterYPixel;
        private double clawCenterXCm;
        private double clawCenterYCm;
        private volatile double moveForward = 0;
        private volatile double moveSideways = 0;
        private volatile double servoPositionOffset = 0;
        private DetectedCube closestCube = null;
        private double distanceToClosestCube = 0;

        public int getClawCenterXPixel() { return clawCenterXPixel; }
        public int getClawCenterYPixel() { return clawCenterYPixel; }
        public double getClawCenterXCm() { return clawCenterXCm; }
        public double getClawCenterYCm() { return clawCenterYCm; }
        public double getMoveForward() { return moveForward; }
        public double getMoveSideways() { return moveSideways; }
        public double getServoPositionOffset() { return servoPositionOffset; }
        public List<DetectedCube> getDetectedCubes() { return detectedCubes; }
        public DetectedCube getClosestCube() { return closestCube; }
        public double getDistanceToClosestCube() { return distanceToClosestCube; }


        @Override
        public Mat processFrame(Mat input) {
            Mat outputImage = input.clone();

            double brightnessFactor = 0.77;
            double contrastFactor = 1.2;

            outputImage.convertTo(outputImage, CvType.CV_8U, contrastFactor, (128 - 128 * contrastFactor) + brightnessFactor * 0);

            detectedCubes.clear();
            closestCube = null;
            distanceToClosestCube = 0;
            resetMovementAndAngle();

            Imgproc.cvtColor(outputImage, hsvImage, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvImage, RED_LOWER_1, RED_UPPER_1, redMask1);
            Core.inRange(hsvImage, RED_LOWER_2, RED_UPPER_2, redMask2);
            Core.bitwise_or(redMask1, redMask2, mask);

            Core.inRange(hsvImage, BLUE_LOWER, BLUE_UPPER, blueMask);
            Core.bitwise_or(mask, blueMask, mask);

            Core.inRange(hsvImage, YELLOW_LOWER, YELLOW_UPPER, yellowMask);
            Core.bitwise_or(mask, yellowMask, mask);


            int width = outputImage.cols();
            int height = outputImage.rows();
            clawCenterXPixel = (int) (CLAW_CENTER_X_CM / PIXELS_TO_CM_RATIO + (double) width / 2);
            clawCenterYPixel = (int) (height - (CLAW_CENTER_Y_CM + CLAW_OFFSET_FROM_CAMERA_CM) / PIXELS_TO_CM_RATIO);
            clawCenterXCm = calculateWorldCoordinatesX(clawCenterXPixel, height);
            clawCenterYCm = calculateWorldCoordinatesY(clawCenterYPixel, height);


            Imgproc.drawMarker(outputImage, new Point(clawCenterXPixel, clawCenterYPixel), new Scalar(0, 255, 255), Imgproc.MARKER_CROSS, 20, 2);

            detectColorRegions(outputImage, outputImage, mask);
            calculateMovementAndServoOffset(outputImage);

            double rawMoveForward = moveForward;
            double rawMoveSideways = moveSideways;

            moveForward = applyMovingAverage(moveForwardHistory, moveForward);
            moveSideways = applyMovingAverage(moveSidewaysHistory, moveSideways);


            redMask1.release();
            redMask2.release();
            blueMask.release();
            yellowMask.release();
            mask.release();
            hsvImage.release();

            telemetry.addData("moveForward_Raw (cm)", String.format(Locale.US, "%.2f", rawMoveForward));
            telemetry.addData("moveSideways_Raw (cm)", String.format(Locale.US, "%.2f", rawMoveSideways));
            telemetry.addData("moveForward_Smoothed (cm)", String.format(Locale.US, "%.2f", moveForward));
            telemetry.addData("moveSideways_Smoothed (cm)", String.format(Locale.US, "%.2f", moveSideways));

            return outputImage;
        }


        private void detectColorRegions(Mat input, Mat outputImage, Mat mask) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Mat localMask = mask.clone();

            try {
                Imgproc.findContours(localMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint contour : contours) {
                    if (Imgproc.contourArea(contour) < 500) continue;

                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    Size size_rect = rect.size;
                    double w_rect = size_rect.width;
                    double h_rect = rect.size.height;
                    double angle_rect = rect.angle;

                    Rect boundingBox = Imgproc.boundingRect(contour);
                    int x = boundingBox.x;
                    int y = boundingBox.y;
                    int w = boundingBox.width;
                    int h = boundingBox.height;

                    if (w < MIN_REGION_WIDTH || h < MIN_REGION_HEIGHT) continue;

                    if (w > SINGLE_SIDE_MAX_SIZE || h > SINGLE_SIDE_MAX_SIZE) {
                        continue;
                    }
                    if (w > DOUBLE_SIDE_MAX_SIZE && h > DOUBLE_SIDE_MAX_SIZE) {
                        continue;
                    }


                    double aspectRatioDetected = 0.0;
                    double angleDegrees = 0.0;

                    if (w_rect > 0 && h_rect > 0) {
                        if (w_rect > h_rect) {
                            aspectRatioDetected = h_rect / w_rect;
                            angleDegrees = angle_rect;
                        } else {
                            aspectRatioDetected = w_rect / h_rect;
                            angleDegrees = angle_rect + 90;
                        }
                        angleDegrees = angleDegrees % 180;
                    }

                    Point center = new Point(x + w / 2.0, y + h / 2.0);
                    DetectedCube cube = createDetectedCube(input, center, angleDegrees, boundingBox, aspectRatioDetected);
                    detectedCubes.add(cube);

                    Imgproc.rectangle(outputImage, boundingBox, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(outputImage, String.format(Locale.US, "%s %.1fdeg", cube.color, angleDegrees),
                            new Point(x, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 0, 0), 2);
                }
                contours.forEach(Mat::release);
            } finally {
                hierarchy.release();
                localMask.release();
            }
        }


        private double calculateAspectRatio(double width, double height) {
            return (width > height) ? height / width : width / height;
        }

        private DetectedCube createDetectedCube(Mat input, Point center, double angleDegrees, Rect boundingBox, double aspectRatioDetected) {
            String color = getColorFromMask((int) center.x, (int) center.y);
            return new DetectedCube(
                    color,
                    (int) center.x,
                    (int) center.y,
                    angleDegrees,
                    boundingBox,
                    aspectRatioDetected,
                    input.cols(),
                    input.rows(),
                    PIXELS_TO_CM_RATIO,
                    CLAW_OFFSET_FROM_CAMERA_CM
            );
        }

        private String getColorFromMask(int x, int y) {
            int regionSize = 10;
            int halfRegionSize = regionSize / 2;
            int redCount = 0;
            int blueCount = 0;
            int yellowCount = 0;

            for (int i = -halfRegionSize; i < halfRegionSize; i++) {
                for (int j = -halfRegionSize; j < halfRegionSize; j++) {
                    int sampleX = x + j;
                    int sampleY = y + i;

                    if (sampleX >= 0 && sampleX < redMask1.cols() && sampleY >= 0 && sampleY < redMask1.rows()) {
                        try {
                            if (redMask1.get(sampleY, sampleX)[0] == 255 || redMask2.get(sampleY, sampleX)[0] == 255) {
                                redCount++;
                            } else if (blueMask.get(sampleY, sampleX)[0] == 255) {
                                blueCount++;
                            } else if (yellowMask.get(sampleY, sampleX)[0] == 255) {
                                yellowCount++;
                            }
                        } catch (Exception e) {
                            continue;
                        }
                    }
                }
            }

            if (redCount >= blueCount && redCount >= yellowCount && redCount > 0) {
                return "red";
            } else if (blueCount >= redCount && blueCount >= yellowCount && blueCount > 0) {
                return "blue";
            } else if (yellowCount >= redCount && yellowCount >= blueCount && yellowCount > 0) {
                return "yellow";
            } else {
                return "unknown";
            }
        }

        private double calculateWorldCoordinatesX(int centerXPixel, int imageHeight) {
            return (centerXPixel - imageHeight / 2.0) * PIXELS_TO_CM_RATIO;
        }

        private double calculateWorldCoordinatesY(int centerYPixel, int imageHeight) {
            int pixelDistanceFromBody = imageHeight - centerYPixel;
            return (pixelDistanceFromBody * PIXELS_TO_CM_RATIO) - CLAW_OFFSET_FROM_CAMERA_CM;
        }


        private void calculateMovementAndServoOffset(Mat outputImage) {
            List<DetectedCube> validCubesAlliance = new ArrayList<>();
            List<DetectedCube> validCubesNeutral = new ArrayList<>();

            for (DetectedCube cube : detectedCubes) {
                if (cube.color.equals("yellow")) {
                    validCubesNeutral.add(cube);
                } else if (cube.color.equalsIgnoreCase(ALLIANCE_COLOR)) {
                    validCubesAlliance.add(cube);
                }
            }

            List<DetectedCube> cubesToConsider = validCubesAlliance.isEmpty() ? validCubesNeutral : validCubesAlliance;
            if (!cubesToConsider.isEmpty()) {
                for (DetectedCube cube : cubesToConsider) {
                    cube.distanceToClawCm = calculateDistance(cube);
                }
                cubesToConsider.sort((c1, c2) -> Double.compare(c1.distanceToClawCm, c2.distanceToClawCm));
                closestCube = cubesToConsider.get(0);
                distanceToClosestCube = closestCube.distanceToClawCm;

                moveForward = closestCube.centerYCm - clawCenterYCm;
                moveSideways = closestCube.centerXCm - clawCenterXCm - CLAW_HORIZONTAL_ERROR_CM;

                double clawAngle_deviation = closestCube.angleDegrees - 90;
                servoPositionOffset = SERVO_ANGLE_COEFFICIENT * DIRECTION_MULTIPLIER_HENG * clawAngle_deviation + ANGLE_OFFSET_HENG;


                Rect bb = closestCube.boundingBox;
                Imgproc.rectangle(outputImage, bb, new Scalar(255, 0, 255), 3);

                int targetClawCenterXPixel = clawCenterXPixel + (int)(moveSideways / PIXELS_TO_CM_RATIO);
                int targetClawCenterYPixel = clawCenterYPixel - (int)(moveForward / PIXELS_TO_CM_RATIO);

                Imgproc.drawMarker(outputImage, new Point(targetClawCenterXPixel, targetClawCenterYPixel), new Scalar(255, 255, 0), Imgproc.MARKER_CROSS, 20, 2);

                Imgproc.line(outputImage, new Point(clawCenterXPixel, clawCenterYPixel), new Point(targetClawCenterXPixel, targetClawCenterYPixel), new Scalar(255, 255, 0), 2);

            } else {
                resetMovementAndAngle();
            }
        }


        private double calculateDistance(DetectedCube cube) {
            double dx = cube.centerXCm - clawCenterXCm;
            double dy = cube.centerYCm - clawCenterYCm;
            return Math.sqrt(dx * dx + dy * dy);
        }

        private void resetMovementAndAngle() {
            moveForward = 0;
            moveSideways = 0;
            servoPositionOffset = 0;
            closestCube = null;
            distanceToClosestCube = 0;
            moveForwardHistory.clear();
            moveSidewaysHistory.clear();
        }


        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) camera.pauseViewport();
            else camera.resumeViewport();
        }

        private double applyMovingAverage(List<Double> history, double newValue) {
            history.add(newValue);

            if (history.size() > MOVING_AVERAGE_WINDOW_SIZE) {
                history.remove(0);
            }

            double sum = 0;
            for (Double value : history) {
                sum += value;
            }

            return sum / history.size();
        }
    }

    public static class DetectedCube {
        String color;
        int centerXImagePx;
        int centerYImagePx;
        double centerXCm;
        double centerYCm;
        double angleDegrees;
        Rect boundingBox;
        double aspectRatio;
        double distanceToClawCm;

        public DetectedCube(String color, int x, int y, double angle, Rect rect, double ratio, int imgWidth, int imgHeight, double pixelsToCmRatio, double clawOffsetFromCameraCm) {
            this.color = color;
            this.centerXImagePx = x;
            this.centerYImagePx = y;
            this.angleDegrees = angle;
            this.boundingBox = rect;
            this.aspectRatio = ratio;
            this.centerXCm = (x - imgWidth / 2.0) * pixelsToCmRatio;
            this.centerYCm = (imgHeight - y) * pixelsToCmRatio - clawOffsetFromCameraCm;
            this.distanceToClawCm = 0;
        }
    }
}