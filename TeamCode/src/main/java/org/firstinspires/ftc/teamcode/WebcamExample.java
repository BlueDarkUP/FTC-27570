/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
public class WebcamExample extends LinearOpMode {

    private enum CameraType {
        INTERNAL_CAMERA_V1,
        INTERNAL_CAMERA_V2,
        WEBCAM
    }

    private static final CameraType CAMERA_TYPE = CameraType.WEBCAM; // 设置摄像头类型
    private OpenCvCamera camera;


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


    private volatile boolean isProcessing = false; // 使用信号量
    private boolean previousBButtonState = false; // 存储上一次按钮状态
    private Pose2d recordedPose = null;
    private volatile boolean cameraInitialized = false;
    private MecanumDrive drive;
    private ColorDetectionPipelineImpl pipeline;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        // 初始化摄像头
        if (!initializeCamera()) {
            return;
        }

        // 如果相机初始化成功，等待开始
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();


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
                double moveForwardinINCH = convertCmToInch(pipeline.getMoveForward());
                double moveSidewaysinINCH = convertCmToInch(pipeline.getMoveSideways());

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

            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());

            telemetry.addData("Move Forward (cm)", "%.1f", pipeline.getMoveForward());
            telemetry.addData("Move Sideways (cm)", "%.1f", pipeline.getMoveSideways());
            telemetry.addData("Claw Angle (deg)", "%.1f", pipeline.getClawAngle());

            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if (gamepad1.a) {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                camera.stopStreaming();
                //camera.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(50);
        }

        //  在 OpMode 结束时关闭摄像头
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }


    private boolean initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        try {
            switch (CAMERA_TYPE) {
                case INTERNAL_CAMERA_V1:
                    camera = OpenCvCameraFactory.getInstance().createInternalCamera(
                            OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                    break;
                case INTERNAL_CAMERA_V2:
                    camera = OpenCvCameraFactory.getInstance().createInternalCamera2(
                            OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
                    break;
                case WEBCAM:
                    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
                    camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
                    break;
            }

            if (camera == null) {
                telemetry.addData("Camera Error", "Failed to create camera instance.");
                telemetry.update();
                return false;
            }

            pipeline = new ColorDetectionPipelineImpl();
            camera.setPipeline(pipeline);


            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                    cameraInitialized = true;
                    telemetry.addData("Camera Status", "Opened");
                    telemetry.update();
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera Error", "Failed to open camera. Code: " + errorCode);
                    telemetry.update();
                }
            });
            // 等待相机初始化
            while (!isStarted() && !cameraInitialized) {
                sleep(100);
            }

            if (!cameraInitialized) {
                telemetry.addData("Camera Status", "Initialization Failed, OpMode Stopped.");
                telemetry.update();
                return false; // 初始化失败，退出 OpMode
            }


        } catch (Exception e) {
            telemetry.addData("Camera Error", "Exception: " + e.getMessage());
            telemetry.update();
            return false;
        }

        return true; // 初始化成功
    }


    // 用于将厘米转换为英寸
    private double convertCmToInch(double cm) {
        return cm * 0.39370;
    }

    /*
     * 一个示例图像处理管道，用于在收到来自摄像头的每个帧时运行。
     * 注意：此管道不是在您的 OpMode 线程上调用的。它是在帧工作线程上调用的。
     */
    class ColorDetectionPipelineImpl extends OpenCvPipeline {
        private final Mat bgrImage = new Mat();  // 用于存储 BGR 图像
        private final Mat hsvImage = new Mat(); // 用于存储 HSV 图像
        private final Mat mask = new Mat();
        private  Mat outputImage = new Mat(); // 用于存储输出图像

        private final List<DetectedCube> detectedCubes = new ArrayList<>();
        private boolean viewportPaused;

        private volatile double moveForward = 0;
        private volatile double moveSideways = 0;
        private volatile double clawAngle = 0;


        @Override
        public void init(Mat firstFrame) {
            outputImage = new Mat(firstFrame.size(), firstFrame.type());
        }

        @Override
        public Mat processFrame(Mat input) {
            if (!isProcessing) {
                input.copyTo(outputImage);
                return outputImage; // 如果没有请求处理，则直接返回输入图像
            }
            Core.flip(input, input, -1); // 翻转图像，因为摄像头是反的
            input.copyTo(outputImage);
            detectedCubes.clear();


            // 颜色空间转换: RGBA -> BGR -> HSV
            Imgproc.cvtColor(input, bgrImage, Imgproc.COLOR_RGBA2BGR);
            Imgproc.cvtColor(bgrImage, hsvImage, Imgproc.COLOR_BGR2HSV);

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

        public double getMoveForward() {
            return moveForward;
        }

        public double getMoveSideways() {
            return moveSideways;
        }

        public double getClawAngle() {
            return clawAngle;
        }
        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) camera.pauseViewport();
            else camera.resumeViewport();
        }


        // 检测到的立方体数据结构
        class DetectedCube {
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
}