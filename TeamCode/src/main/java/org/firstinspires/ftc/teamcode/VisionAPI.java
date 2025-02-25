package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class VisionAPI {
    private static OpenCvCamera camera;
    private static ColorDetectionPipelineImpl colorDetectionPipeline;
    private static final int cameraWidth = 1280;
    private static final int cameraHeight = 720;
    private static boolean isCameraInitialized = false;
    private static Telemetry telemetryStatic; // Static telemetry
    private static WebcamName webcamNameStatic; // Static webcam name

    // Constants from WebcamExample.java - made static here
    private static final double PIXELS_TO_CM_RATIO = 0.021875;
    private static final double CLAW_OFFSET_FROM_CAMERA_CM = 0;
    public static String ALLIANCE_COLOR = "yellow"; // Changed to public static, not final
    private static final int MIN_REGION_WIDTH = 10;
    private static final int MIN_REGION_HEIGHT = 10;
    private static final double CLAW_CENTER_X_CM = -0.5;
    private static final double CLAW_CENTER_Y_CM = 4.64;
    public static final double SERVO_CENTER_POSITION_HENG = 0.54; // Default ServoPositions.CLAW_HENG_DEFAULT; - made public static and initialized directly
    private static final double CLAW_HORIZONTAL_ERROR_CM = -5.7;

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


    public static class ApproachData {
        public double distanceToCubeCm;
        public double moveSidewaysCm;
        public double servoPositionOffset;
        public double moveForwardcm;

        public ApproachData() {
            this.moveForwardcm = 0;
            this.moveSidewaysCm = 0;
            this.servoPositionOffset = 0;
        }
    }

    public static class GrabData {
        public double moveForwardCm;
        public double moveSidewaysCm;
        public double servoPosition;

        public GrabData() {
            this.moveForwardCm = 0;
            this.moveSidewaysCm = 0;
            this.servoPosition = 0;
        }
    }


    public static boolean initialize(WebcamName webcamName, Telemetry telemetry) {
        webcamNameStatic = webcamName; // Store static webcam name
        telemetryStatic = telemetry; // Store static telemetry
        telemetryStatic.addLine("VisionAPI 初始化摄像头...");
        telemetryStatic.update();
        // 使用 telemetryStatic.hardwareMap.appContext 获取 Context
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        try {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamNameStatic, cameraMonitorViewId);
            if (camera == null) {
                telemetryStatic.addData("VisionAPI 摄像头错误", "创建摄像头实例失败 (camera == null).");
                telemetryStatic.update();
                return false;
            }

            colorDetectionPipeline = new ColorDetectionPipelineImpl();
            camera.setPipeline(colorDetectionPipeline);

            telemetryStatic.addLine("VisionAPI 尝试打开摄像头设备异步...");
            telemetryStatic.update();

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    telemetryStatic.addLine("VisionAPI 摄像头设备已打开 (onOpened 回调).");
                    telemetryStatic.update();
                    camera.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                    isCameraInitialized = true;
                    telemetryStatic.addData("VisionAPI 摄像头状态", "已打开并开始推流");
                    telemetryStatic.update();
                }

                @Override
                public void onError(int errorCode) {
                    telemetryStatic.addData("VisionAPI 摄像头错误", "打开摄像头设备失败 (onError 回调).");
                    telemetryStatic.addData("VisionAPI 错误代码", errorCode);
                    telemetryStatic.update();
                    isCameraInitialized = true;
                }
            });

            telemetryStatic.addLine("VisionAPI 等待摄像头初始化完成...");
            telemetryStatic.update();
            long startTime = System.currentTimeMillis();
            while (!isCameraInitialized) {
                if (System.currentTimeMillis() - startTime > 5000) { // Timeout after 5 seconds
                    telemetryStatic.addData("VisionAPI 摄像头状态", "初始化超时.");
                    telemetryStatic.update();
                    return false; // Initialization timed out
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return false;
                }
            }

            if (!isCameraInitialized) {
                telemetryStatic.addData("VisionAPI 摄像头状态", "初始化失败, VisionAPI 初始化中止.");
                telemetryStatic.update();
                return false;
            }
            telemetryStatic.addLine("VisionAPI 摄像头初始化完成");
            telemetryStatic.update();
            return true;

        } catch (Exception e) {
            telemetryStatic.addData("VisionAPI 摄像头错误", "初始化异常: " + e.getMessage());
            telemetryStatic.update();
            return false;
        }
    }

    public static ApproachData getApproachData() {
        ApproachData approachData = new ApproachData();
        DetectedCube closestCube = null;
        double moveForwardcm = 0;
        double moveSidewaysCm = 0;
        double servoPositionOffset = 0;

        List<DetectedCube> cubes;
        synchronized (colorDetectionPipeline) {
            cubes = colorDetectionPipeline.getDetectedCubes();
            closestCube = colorDetectionPipeline.getClosestCube();
            moveForwardcm = colorDetectionPipeline.getMoveForward();
            moveSidewaysCm = colorDetectionPipeline.getMoveSideways();
            servoPositionOffset = colorDetectionPipeline.getServoPositionOffset();
        }

        if (closestCube != null) {
            approachData.moveForwardcm = moveForwardcm;
            approachData.moveSidewaysCm = moveSidewaysCm;
            approachData.servoPositionOffset = servoPositionOffset;
        } else {
            approachData.distanceToCubeCm = -1; // Indicate no cube detected
            approachData.moveSidewaysCm = 0;
            approachData.servoPositionOffset = 0;
        }
        return approachData;
    }

    public static GrabData getGrabData() {
        GrabData grabData = new GrabData();
        double moveForwardCm = 0;
        double moveSidewaysCm = 0;
        double servoPosition = 0;

        List<DetectedCube> cubes;
        synchronized (colorDetectionPipeline) {
            cubes = colorDetectionPipeline.getDetectedCubes();
            moveForwardCm = colorDetectionPipeline.getMoveForward();
            moveSidewaysCm = colorDetectionPipeline.getMoveSideways();
            servoPosition = SERVO_CENTER_POSITION_HENG + colorDetectionPipeline.getServoPositionOffset();
            servoPosition = wrapAroundServoValue(servoPosition);
        }

        grabData.moveForwardCm = moveForwardCm;
        grabData.moveSidewaysCm = moveSidewaysCm;
        grabData.servoPosition = servoPosition;
        return grabData;
    }


    public static void stopCamera() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
            telemetryStatic.addLine("VisionAPI 摄像头流已停止且设备已关闭");
            telemetryStatic.update();
            isCameraInitialized = false; // Reset flag for potential re-initialization
        }
    }

    // Moved wrapAroundServoValue method to VisionAPI class and made static
    public static double wrapAroundServoValue(double value) {
        while (value > 1) {
            value -= 1;
        }
        while (value < 0) {
            value += 1;
        }
        return value;
    }


    // -------------------- Inner Classes (Now static inner classes) --------------------

    public static class ColorDetectionPipelineImpl extends OpenCvPipeline {
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

        public double getMoveForward() { return moveForward; }
        public double getMoveSideways() { return moveSideways; }
        public double getServoPositionOffset() { return servoPositionOffset; }
        public synchronized List<DetectedCube> getDetectedCubes() {
            return new ArrayList<>(detectedCubes);
        }
        public synchronized DetectedCube getClosestCube() {
            return closestCube;
        }
        public double getDistanceToClosestCube() { return distanceToClosestCube; }


        @Override
        public Mat processFrame(Mat input) {
            Mat outputImage = input.clone();

            double brightnessFactor = 0.77;
            double contrastFactor = 1.2;

            outputImage.convertTo(outputImage, CvType.CV_8U, contrastFactor, (128 - 128 * contrastFactor) + brightnessFactor * 0);


            synchronized (this) {
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
            }
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

            if (telemetryStatic != null) { // Check if telemetry is available before using
                telemetryStatic.addData("moveForward_Raw (cm)", String.format(Locale.US, "%.2f", rawMoveForward));
                telemetryStatic.addData("moveSideways_Raw (cm)", String.format(Locale.US, "%.2f", rawMoveSideways));
                telemetryStatic.addData("moveForward_Smoothed (cm)", String.format(Locale.US, "%.2f", moveForward));
                telemetryStatic.addData("moveSideways_Smoothed (cm)", String.format(Locale.US, "%.2f", moveSideways));
            }


            return outputImage;
        }


        private void detectColorRegions(Mat input, Mat outputImage, Mat mask) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Mat localMask = mask.clone();

            try {
                Imgproc.findContours(localMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint contour : contours) {
                    if (Imgproc.contourArea(contour) < 500) continue; // 移除或者注释掉这行

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
                    synchronized (this) {
                        detectedCubes.add(cube);
                    }

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
            int totalPixels = regionSize * regionSize;

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
            List<DetectedCube> currentDetectedCubes;

            synchronized (this) {
                currentDetectedCubes = new ArrayList<>(detectedCubes);
            }


            for (DetectedCube cube : currentDetectedCubes) {
                if (cube.color.equals("yellow")) {
                    validCubesNeutral.add(cube);
                } else if (cube.color.equalsIgnoreCase(VisionAPI.ALLIANCE_COLOR)) { // Use VisionAPI.ALLIANCE_COLOR here
                    validCubesAlliance.add(cube);
                }
            }

            List<DetectedCube> cubesToConsider = validCubesAlliance.isEmpty() ? validCubesNeutral : validCubesAlliance;
            if (!cubesToConsider.isEmpty()) {
                for (DetectedCube cube : cubesToConsider) {
                    cube.distanceToClawCm = calculateDistance(cube);
                }
                cubesToConsider.sort((c1, c2) -> Double.compare(c1.distanceToClawCm, c2.distanceToClawCm));
                synchronized (this) {
                    closestCube = cubesToConsider.get(0);
                }
                distanceToClosestCube = closestCube.distanceToClawCm;

                moveForward = closestCube.centerYCm - clawCenterYCm;
                moveSideways = closestCube.centerXCm - clawCenterXCm - CLAW_HORIZONTAL_ERROR_CM;

                double clawAngleDegrees = closestCube.angleDegrees; // 获取检测到的方块角度

                // 新的舵机数值计算算法
                double angleDeviation = clawAngleDegrees + 90; // 角度偏差
                double servoValueChange = angleDeviation / 180.0; // 舵机数值变化量
                double servoValue = SERVO_CENTER_POSITION_HENG - servoValueChange; // 计算舵机数值

                // 使用 wrap-around 算法, now calling the static method in the outer class
                servoValue = VisionAPI.wrapAroundServoValue(servoValue);
                servoPositionOffset = servoValue - SERVO_CENTER_POSITION_HENG; // 计算相对于中心位置的偏移量


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

        private synchronized void resetMovementAndAngle() {
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