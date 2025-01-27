package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AutonomousVisionOpMode")
public class AutonomousVisionOpMode extends LinearOpMode {

    // --- 可调参数 ---
    private static final double PIXELS_TO_CM_RATIO = 0.01;
    private static final double CLAW_OFFSET_FROM_CAMERA_CM = 0;
    private static final String ALLIANCE_COLOR = "yellow";
    private static final int MIN_REGION_WIDTH = 20; // 最小区域宽度 (像素) - 用于过滤小框
    private static final int MIN_REGION_HEIGHT = 20; // 最小区域高度 (像素) - 用于过滤小框
    private static final double EXPECTED_ASPECT_RATIO = 3.0 / 7.0;
    private static final double ASPECT_RATIO_TOLERANCE = 0.3;
    private static final double CLAW_CENTER_X_CM = 0;
    private static final double CLAW_CENTER_Y_CM = 0;

    // --- 颜色阈值 ---
    private static final Scalar[] RED_LOWER = {new Scalar(170, 130, 155), new Scalar(0, 135, 160)};
    private static final Scalar[] RED_UPPER = {new Scalar(180, 255, 255), new Scalar(10, 255, 255)};
    private static final Scalar BLUE_LOWER = new Scalar(100, 120, 210);
    private static final Scalar BLUE_UPPER = new Scalar(130, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(20, 150, 180);
    private static final Scalar YELLOW_UPPER = new Scalar(40, 255, 255);

    // 使用volatile保证多线程可见性
    private volatile double moveForward = 0;
    private volatile double moveSideways = 0;
    private volatile double clawAngle = 0;

    private volatile boolean isProcessing = false; // 使用信号量
    private boolean previousBButtonState = false; // 存储上一次按钮状态

    private OpenCvWebcam webcam;
    private Pose2d recordedPose = null;
    private volatile boolean cameraInitialized = false; // 标志相机是否初始化成功

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ColorDetectionPipelineImpl pipeline = new ColorDetectionPipelineImpl();
        webcam.setPipeline(pipeline);

        // 设置相机超时时间和重试机制
        final int MAX_RETRIES = 3;
        int retries = 0;
        webcam.setMillisecondsPermissionTimeout(5000); //设置5秒的超时时间

        while (retries < MAX_RETRIES && !isStarted()) {
            // 使用 try-catch 块捕获相机初始化异常
            try {
                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        // 相机成功打开，开始预览
                        webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                        telemetry.addData("Camera Status", "Opened");
                        telemetry.update();
                        cameraInitialized = true; // 设置初始化标志
                    }

                    @Override
                    public void onError(int errorCode) {
                        // 相机打开失败，输出错误信息
                        telemetry.addData("Camera Error", "Code: " + errorCode);
                        telemetry.update();
                    }
                });

                // 如果相机开始预览，退出循环
                if(cameraInitialized) {
                    break;
                }

            }  catch (Exception e) {
                //捕获其他异常
                telemetry.addData("Camera Error", "Exception: " + e.getMessage());
                telemetry.update();
            }

            if(!isStarted()) {
                //等待 1 秒后重试
                sleep(1000);
                retries++;
                if(retries >= MAX_RETRIES){
                    telemetry.addData("Camera Error", "Max retries reached, exiting");
                    telemetry.update();
                    requestOpModeStop();
                    break;
                }
            } else {
                break;
            }
        }


        // 如果相机初始化失败，则退出 OpMode
        if(!isStarted() || !cameraInitialized) {
            telemetry.addData("Camera Status", "Initialization Failed, OpMode Stopped.");
            telemetry.update();
            return;
        } else{
            // 相机初始化成功，等待开始
            waitForStart();
        }

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            // 检测B键是否被按下
            boolean currentBButtonState = gamepad1.b;
            if (currentBButtonState && !previousBButtonState) {
                //保存当前机器人位置
                recordedPose = drive.pose;
                telemetry.addData("Recorded Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                        recordedPose.position.x, recordedPose.position.y, Math.toDegrees(recordedPose.heading.toDouble()));
                telemetry.update();
                isProcessing = true;
            }
            previousBButtonState = currentBButtonState; // 保存当前按钮状态

            if (isProcessing) {
                //移动逻辑
                double moveForwardinINCH = convertCmToInch(moveForward);
                double moveSidewaysinINCH = convertCmToInch(moveSideways);

                try {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(new Vector2d(recordedPose.position.x + moveForwardinINCH, recordedPose.position.y - moveSidewaysinINCH), 0)
                                    .build()
                    );
                } catch (Exception e) {
                    telemetry.addData("Movement Error", "Exception: " + e.getMessage());
                    telemetry.update();
                }


                isProcessing = false; //处理完成后重置状态
            }

            telemetry.addData("Move Forward (cm)", "%.1f", moveForward);
            telemetry.addData("Move Sideways (cm)", "%.1f", moveSideways);
            telemetry.addData("Claw Angle (deg)", "%.1f", clawAngle);
            telemetry.update();
            sleep(50);
        }
        webcam.stopStreaming();
    }

    // 用于将厘米转换为英寸
    private double convertCmToInch(double cm) {
        return cm * 0.39370;
    }

    class ColorDetectionPipelineImpl extends OpenCvPipeline {
        private final Mat hsvImage = new Mat();
        private final Mat mask = new Mat();
        private final Mat outputImage = new Mat();
        private final List<DetectedCube> detectedCubes = new ArrayList<>();
        private boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            if (!isProcessing) {
                input.copyTo(outputImage);
                return outputImage; // 如果没有请求处理，则直接返回输入图像
            }
            Core.flip(input, input, -1); // 翻转图像，因为摄像头是反的
            input.copyTo(outputImage);
            detectedCubes.clear();

            // 颜色空间转换
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

            // 创建颜色掩码
            Mat redMask = createMask(hsvImage, RED_LOWER, RED_UPPER, "red");
            Mat blueMask = createMask(hsvImage, new Scalar[]{BLUE_LOWER}, new Scalar[]{BLUE_UPPER}, "blue");
            Mat yellowMask = createMask(hsvImage, new Scalar[]{YELLOW_LOWER}, new Scalar[]{YELLOW_UPPER}, "yellow");

            // 合并掩码
            Core.bitwise_or(redMask, blueMask, mask);
            Core.bitwise_or(mask, yellowMask, mask);

            // 释放临时掩码
            redMask.release();
            blueMask.release();
            yellowMask.release();

            // 检测处理
            detectColorRegions(input, outputImage, mask);
            calculateMovementAndClawAngle(outputImage);

            return outputImage;
        }

        // 创建颜色掩码
        private Mat createMask(Mat hsvImage, Scalar[] lowerBounds, Scalar[] upperBounds, String colorName) {
            Mat colorMask = new Mat(hsvImage.size(), CvType.CV_8UC1, Scalar.all(0));
            for (int i = 0; i < lowerBounds.length; i++) {
                Mat tempMask = new Mat();
                Core.inRange(hsvImage, lowerBounds[i], upperBounds[i], tempMask);
                Core.bitwise_or(colorMask, tempMask, colorMask);
                tempMask.release();
            }
            return colorMask;
        }

        private void detectColorRegions(Mat input, Mat outputImage, Mat mask) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            try {
                // 查找轮廓
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area < 500) continue;

                    // 获取最小外接矩形
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    RotatedRect rect = Imgproc.minAreaRect(contour2f);
                    contour2f.release();

                    // 获取包围盒
                    Rect boundingBox = Imgproc.boundingRect(contour);
                    // 过滤小框 - 新增代码
                    if (boundingBox.width < MIN_REGION_WIDTH || boundingBox.height < MIN_REGION_HEIGHT) {
                        continue; // 如果宽度或高度小于最小值，则跳过此轮廓
                    }

                    // 计算宽高比
                    float width = (float) rect.size.width;
                    float height = (float) rect.size.height;
                    double aspectRatio = (width > height) ? height / width : width / height;

                    // 宽高比过滤
                    if (Math.abs(aspectRatio - EXPECTED_ASPECT_RATIO) > ASPECT_RATIO_TOLERANCE) continue;

                    // 获取中心坐标
                    Point center = rect.center;
                    int centerX = (int) center.x;
                    int centerY = (int) center.y;

                    // 计算世界坐标
                    double[] worldCoords = calculateWorldCoordinates(centerX, centerY, input.cols(), input.rows());
                    // 保存检测结果
                    detectedCubes.add(new DetectedCube(
                            getColorFromMask(mask, centerX, centerY),
                            centerX,
                            centerY,
                            worldCoords[0],
                            worldCoords[1],
                            rect.angle,
                            boundingBox, // 使用包围盒
                            aspectRatio
                    ));

                    // 可视化
                    Imgproc.drawContours(outputImage, List.of(contour), -1, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(outputImage, "Cube", new Point(centerX - 20, centerY - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 1);
                }
            } finally {
                hierarchy.release(); // 确保释放资源
            }
        }

        private String getColorFromMask(Mat mask, int x, int y) {
            // 根据掩码像素值判断颜色
            Scalar pixelColor = new Scalar(0, 0, 0);
            try {
                pixelColor = new Scalar(mask.get(y, x));
            } catch (Exception e) {
                return "unknown";
            }

            if (pixelColor.val[0] == 255) {
                return ALLIANCE_COLOR;
            } else if (pixelColor.val[0] == 0) {
                return "unknow";
            }
            return "unknow";
        }

        // 计算移动和爪子角度
        private void calculateMovementAndClawAngle(Mat outputImage) {
            int width = outputImage.cols();
            int height = outputImage.rows();
            // 计算爪子固定位置
            int clawCenterX = (int) (CLAW_CENTER_X_CM / PIXELS_TO_CM_RATIO + (double) width / 2);
            int clawCenterY = (int) (height - (CLAW_CENTER_Y_CM + CLAW_OFFSET_FROM_CAMERA_CM) / PIXELS_TO_CM_RATIO);
            Imgproc.drawMarker(outputImage, new Point(clawCenterX, clawCenterY),
                    new Scalar(0, 255, 255), Imgproc.MARKER_CROSS, 20, 2);

            // 寻找最近目标
            DetectedCube target = null;
            double minDistance = Double.MAX_VALUE;

            for (DetectedCube cube : detectedCubes) {
                // 优先联盟颜色
                if (!cube.color.equals(ALLIANCE_COLOR) && !cube.color.equals("yellow")) continue;

                double dx = cube.centerXCm - CLAW_CENTER_X_CM;
                double dy = cube.centerYCm - CLAW_CENTER_Y_CM;
                double distance = Math.hypot(dx, dy);

                if (distance < minDistance) {
                    minDistance = distance;
                    target = cube;
                }
            }

            if (target != null) {
                moveForward = target.centerYCm - CLAW_CENTER_Y_CM;
                moveSideways = target.centerXCm - CLAW_CENTER_X_CM;
                clawAngle = -target.angleDegrees;

                // 绘制目标指示
                Imgproc.line(outputImage,
                        new Point(clawCenterX, clawCenterY),
                        new Point(target.centerXImagePx, target.centerYImagePx),
                        new Scalar(255, 0, 255), 2);
            } else {
                moveForward = 0;
                moveSideways = 0;
                clawAngle = 0;
            }
        }

        // 计算世界坐标
        private double[] calculateWorldCoordinates(int x, int y, int imgWidth, int imgHeight) {
            // 使用图像宽度计算水平坐标
            double xCm = (x - imgWidth / 2.0) * PIXELS_TO_CM_RATIO;
            double yCm = (imgHeight - y) * PIXELS_TO_CM_RATIO - CLAW_OFFSET_FROM_CAMERA_CM;
            return new double[]{xCm, yCm};
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) webcam.pauseViewport();
            else webcam.resumeViewport();
        }
    }

    // 检测到的立方体数据结构
    static class DetectedCube {
        String color;
        int centerXImagePx;
        int centerYImagePx;
        double centerXCm;
        double centerYCm;
        double angleDegrees;
        Rect boundingBox;
        double aspectRatio;

        public DetectedCube(String color, int x, int y, double xCm, double yCm,
                            double angle, Rect rect, double ratio) {
            this.color = color;
            this.centerXImagePx = x;
            this.centerYImagePx = y;
            this.centerXCm = xCm;
            this.centerYCm = yCm;
            this.angleDegrees = angle;
            this.boundingBox = rect;
            this.aspectRatio = ratio;
        }
    }
}