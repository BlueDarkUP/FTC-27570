package org.firstinspires.ftc.teamcode;
import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "优化后的视觉程序")
public class VisionMovementOutput extends LinearOpMode {
    private MecanumDrive drive;

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

    private static final double SERVO_CENTER_POSITION_HENG = ServoPositions.CLAW_HENG_DEFAULT;
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

    private static class ServoPositions {
        static final double ARM_FORWARD_DEFAULT = 0.65;
        static final double CLAW_SHU_DEFAULT = 0.32;
        static final double CLAW_HENG_DEFAULT = 0.54;
        static final double FORWARD_SLIDE_DEFAULT = 0;
        static final double FORWARD_SLIDE_2_DEFAULT = 0.37; // 添加 forward_slide_2 的默认位置为 0
        static final double ARM_FORWARD_OVERRIDE = 0.4;
        static final double CLAW_SHU_OVERRIDE = 1.0;
        static final double FORWARD_SLIDE_OVERRIDE = 0;
        static final double FORWARD_CLAW_DEFAULT = 0.85;
        static final double FORWARD_CLAW_GRAB = 0.0;
    }

    private OpenCvCamera camera;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private Servo armForwardServo, clawHengServo, clawShuServo, forwardSlideServo, forwardSlide2Servo, forwardClawServo; // 添加 forwardSlide2Servo

    private ColorDetectionPipelineImpl colorDetectionPipeline;
    private final int cameraWidth = 1280;
    private final int cameraHeight = 720;

    private volatile boolean isCameraInitialized = false;
    private boolean processFrameFlag = false;
    private boolean isApproached = false;
    private boolean isStreaming = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        initializeHardware();
        initializeServos(); // 确保在 initializeHardware 中调用 initializeServos
        if (!initializeCamera()) {
            return;
        }
        setServosToDefaultPosition();

        telemetry.addLine("等待开始");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            handleGamepadInput();
            updateTelemetry();
            sleep(50);
        }
        stopCamera();
    }

    private void initializeHardware() {
        initializeMotors();
        initializeServos(); // 确保在这里调用 initializeServos
    }

    private void initializeMotors() {
        telemetry.addLine("初始化电机...");
        telemetry.update();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addLine("电机初始化完成");
    }


    private void initializeServos() {
        telemetry.addLine("初始化舵机...");
        telemetry.update();
        try {
            armForwardServo = hardwareMap.get(Servo.class, "arm_forward");
            clawShuServo = hardwareMap.get(Servo.class, "claw_shu");
            clawHengServo = hardwareMap.get(Servo.class, "claw_heng");
            forwardSlideServo = hardwareMap.get(Servo.class, "forward_slide");
            forwardSlide2Servo = hardwareMap.get(Servo.class, "forward_slide_2"); // 初始化 forward_slide_2
            forwardClawServo = hardwareMap.get(Servo.class, "forward_claw");
            telemetry.addLine("舵机初始化完成");
        } catch (Exception e) {
            telemetry.addData("舵机错误", "获取舵机硬件失败: " + e.getMessage());
            telemetry.update();
            stop();
        }
    }

    private void setServosToDefaultPosition() {
        armForwardServo.setPosition(ServoPositions.ARM_FORWARD_DEFAULT);
        clawShuServo.setPosition(ServoPositions.CLAW_SHU_DEFAULT);
        clawHengServo.setPosition(ServoPositions.CLAW_HENG_DEFAULT);
        forwardSlideServo.setPosition(ServoPositions.FORWARD_SLIDE_DEFAULT);
        forwardSlide2Servo.setPosition(ServoPositions.FORWARD_SLIDE_2_DEFAULT); // 设置 forward_slide_2 的默认位置为 0
        forwardClawServo.setPosition(ServoPositions.FORWARD_CLAW_DEFAULT);
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
                    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
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

    private void handleGamepadInput() {
        if (gamepad1.circle) {
            processFrameAndControlServos();
        } else {
            driveRobotManually();
        }
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

                    armForwardServo.setPosition(ServoPositions.ARM_FORWARD_OVERRIDE);
                    clawHengServo.setPosition(targetHengServoPosition);
                    forwardSlideServo.setPosition(ServoPositions.FORWARD_SLIDE_OVERRIDE);
                    forwardSlide2Servo.setPosition(ServoPositions.FORWARD_SLIDE_2_DEFAULT); // 确保 forward_slide_2 保持在 0
                    clawShuServo.setPosition(1);

                    sleep(300);
                    armForwardServo.setPosition(0.15);
                    sleep(300);
                    forwardClawServo.setPosition(ServoPositions.FORWARD_CLAW_GRAB);
                    boolean isClawDown = true;
                    sleep(300);
                    armForwardServo.setPosition(0.4);
                    sleep(1000);
                    armForwardServo.setPosition(ServoPositions.ARM_FORWARD_DEFAULT);
                    clawShuServo.setPosition(ServoPositions.CLAW_SHU_DEFAULT);
                    clawHengServo.setPosition(ServoPositions.CLAW_HENG_DEFAULT);
                    forwardSlideServo.setPosition(ServoPositions.FORWARD_SLIDE_DEFAULT);
                    forwardSlide2Servo.setPosition(ServoPositions.FORWARD_SLIDE_2_DEFAULT); // 再次确保 forward_slide_2 保持在 0
                    forwardClawServo.setPosition(ServoPositions.FORWARD_CLAW_DEFAULT);
                }
            }


            telemetry.update();
            while (gamepad1.circle && opModeIsActive()) {
                sleep(50);
            }
            processFrameFlag = false;
        }
    }

    private void driveRobotManually() {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        telemetry.addLine("持续处理图像，按下圆形按钮来执行基于视觉的动作");
        if (isStreaming) {
            telemetry.addData("帧计数", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("总帧时间 ms", camera.getTotalFrameTimeMs());
            telemetry.addData("管道处理时间 ms", camera.getPipelineTimeMs());
            telemetry.addData("开销时间 ms", camera.getOverheadTimeMs());
            telemetry.addData("理论最大 FPS", camera.getCurrentPipelineMaxFps());
        } else {
            telemetry.addLine("摄像头推流已停止");
        }


        telemetry.addLine("--- Road Runner 位姿 ---");
        telemetry.addData("X 位置 (英寸)", String.format("%.2f", drive.pose.position.x));
        telemetry.addData("Y 位置 (英寸)", String.format("%.2f", drive.pose.position.y));
        telemetry.addData("Heading (度)", String.format("%.2f", Math.toDegrees(drive.pose.heading.toDouble())));

        telemetry.addLine("--- 视觉检测数据 (持续更新) ---");
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
            telemetry.addLine("未检测到方块 (持续更新)");
        }

        int clawCenterXPixel = colorDetectionPipeline.getClawCenterXPixel();
        int clawCenterYPixel = colorDetectionPipeline.getClawCenterYPixel();
        double clawCenterXCm = colorDetectionPipeline.getClawCenterXCm();
        double clawCenterYCm = colorDetectionPipeline.getClawCenterYCm();
        double moveForward = colorDetectionPipeline.getMoveForward();
        double moveSideways = colorDetectionPipeline.getMoveSideways();
        double servoPositionOffset = colorDetectionPipeline.getServoPositionOffset();

        telemetry.addLine("--- 当前爪子位置 ---");
        telemetry.addData("爪子图像坐标 (px)", String.format(Locale.US, "(%d, %d)", clawCenterXPixel, clawCenterYPixel));
        telemetry.addData("爪子世界坐标 (cm)", String.format(Locale.US, "(%.1f, %.1f)", clawCenterXCm, clawCenterYCm));

        telemetry.addLine("--- 运动指令 ---");
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
                telemetry.addLine("未找到有效目标方块");
            }
        } else {
            telemetry.addLine("未检测到长方体，无运动指令");
        }


        if (gamepad1.a) {
            telemetry.addLine("*** 手动按下 'A' 按钮停止推流 ***");
        }
        telemetry.addData("claw_heng 目标位置", String.format("%.3f", (SERVO_CENTER_POSITION_HENG + colorDetectionPipeline.getServoPositionOffset())));
        telemetry.addData("claw_heng 当前位置", String.format("%.3f", clawHengServo.getPosition()));
        telemetry.addData("forward_slide_2 位置", String.format("%.3f", forwardSlide2Servo.getPosition())); // 添加 forward_slide_2 的遥测数据

        telemetry.update();
    }

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
                    double h_rect = size_rect.height;
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