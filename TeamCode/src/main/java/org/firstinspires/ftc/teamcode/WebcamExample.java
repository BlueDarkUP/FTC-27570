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

    private static final CameraType CAMERA_TYPE = CameraType.WEBCAM;
    private OpenCvCamera camera;

    private static final double PIXELS_TO_CM_RATIO = 0.0183;
    private static final double CLAW_OFFSET_FROM_CAMERA_CM = 0;
    private static final String ALLIANCE_COLOR = "yellow";
    private static final int MIN_REGION_WIDTH = 80;
    private static final int MIN_REGION_HEIGHT = 80;
    private static final double EXPECTED_ASPECT_RATIO = 3.0 / 7.0;
    private static final double ASPECT_RATIO_TOLERANCE = 0.3;
    private static final double CLAW_CENTER_X_CM = 0;
    private static final double CLAW_CENTER_Y_CM = 0;

    private static final Scalar[] RED_LOWER = {new Scalar(170, 130, 155), new Scalar(0, 135, 160)};
    private static final Scalar[] RED_UPPER = {new Scalar(180, 255, 255), new Scalar(10, 255, 255)};
    private static final Scalar BLUE_LOWER = new Scalar(100, 120, 210);
    private static final Scalar BLUE_UPPER = new Scalar(130, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(20, 150, 180);
    private static final Scalar YELLOW_UPPER = new Scalar(40, 255, 255);


    private volatile boolean isProcessing = false;
    private boolean previousBButtonState = false;
    private Pose2d recordedPose = null;
    private volatile boolean cameraInitialized = false;
    private MecanumDrive drive;
    private ColorDetectionPipelineImpl pipeline;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        if (!initializeCamera()) {
            return;
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            boolean currentBButtonState = gamepad1.b;
            if (currentBButtonState && !previousBButtonState) {
                recordedPose = drive.pose;
                telemetry.addData("Recorded Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                        recordedPose.position.x, recordedPose.position.y, Math.toDegrees(recordedPose.heading.toDouble()));
                telemetry.update();
                isProcessing = true;
            }
            previousBButtonState = currentBButtonState;

            if (isProcessing) {
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


                isProcessing = false;
            }

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

            if (gamepad1.a) {
                camera.stopStreaming();
            }

            sleep(50);
        }

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
                    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
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
            while (!isStarted() && !cameraInitialized) {
                sleep(100);
            }

            if (!cameraInitialized) {
                telemetry.addData("Camera Status", "Initialization Failed, OpMode Stopped.");
                telemetry.update();
                return false;
            }


        } catch (Exception e) {
            telemetry.addData("Camera Error", "Exception: " + e.getMessage());
            telemetry.update();
            return false;
        }

        return true;
    }


    private double convertCmToInch(double cm) {
        return cm * 0.39370;
    }


    class ColorDetectionPipelineImpl extends OpenCvPipeline {
        private final Mat bgrImage = new Mat();
        private final Mat hsvImage = new Mat();
        private final Mat mask = new Mat();
        private  Mat outputImage;

        private final List<DetectedCube> detectedCubes = new ArrayList<>();
        private boolean viewportPaused;

        private volatile double moveForward = 0;
        private volatile double moveSideways = 0;
        private volatile double clawAngle = 0;

        public ColorDetectionPipelineImpl() {
            this.outputImage = new Mat();
        }

        @Override
        public void init(Mat firstFrame) {
            outputImage = new Mat(firstFrame.size(), firstFrame.type());
        }


        @Override
        public Mat processFrame(Mat input) {
            if (!isProcessing) {
                input.copyTo(outputImage);
                return outputImage;
            }
            Core.flip(input, input, -1);
            input.copyTo(outputImage);
            detectedCubes.clear();

            Imgproc.cvtColor(input, bgrImage, Imgproc.COLOR_RGBA2BGR);
            Imgproc.cvtColor(bgrImage, hsvImage, Imgproc.COLOR_BGR2HSV);

            Mat redMask = createMask(hsvImage, RED_LOWER, RED_UPPER, "red");
            Mat blueMask = createMask(hsvImage, new Scalar[]{BLUE_LOWER}, new Scalar[]{BLUE_UPPER}, "blue");
            Mat yellowMask = createMask(hsvImage, new Scalar[]{YELLOW_LOWER}, new Scalar[]{YELLOW_UPPER}, "yellow");

            Core.bitwise_or(redMask, blueMask, mask);
            Core.bitwise_or(mask, yellowMask, mask);

            redMask.release();
            blueMask.release();
            yellowMask.release();

            detectColorRegions(input, outputImage, mask);
            calculateMovementAndClawAngle(outputImage);


            return outputImage;
        }

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
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area < 500) continue;

                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    RotatedRect rect = Imgproc.minAreaRect(contour2f);
                    contour2f.release();

                    Rect boundingBox = Imgproc.boundingRect(contour);
                    if (boundingBox.width < MIN_REGION_WIDTH || boundingBox.height < MIN_REGION_HEIGHT) {
                        continue;
                    }

                    float width = (float) rect.size.width;
                    float height = (float) rect.size.height;
                    double aspectRatio = (width > height) ? height / width : width / height;

                    if (Math.abs(aspectRatio - EXPECTED_ASPECT_RATIO) > ASPECT_RATIO_TOLERANCE) continue;

                    Point center = rect.center;
                    int centerX = (int) center.x;
                    int centerY = (int) center.y;

                    double[] worldCoords = calculateWorldCoordinates(centerX, centerY, input.cols(), input.rows());
                    detectedCubes.add(new DetectedCube(
                            getColorFromMask(mask, centerX, centerY),
                            centerX,
                            centerY,
                            worldCoords[0],
                            worldCoords[1],
                            rect.angle,
                            boundingBox,
                            aspectRatio
                    ));

                    Imgproc.drawContours(outputImage, List.of(contour), -1, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(outputImage, "Cube", new Point(centerX - 20, centerY - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 1);
                }
            } finally {
                hierarchy.release();
            }
        }


        private String getColorFromMask(Mat mask, int x, int y) {
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

        private void calculateMovementAndClawAngle(Mat outputImage) {
            int width = outputImage.cols();
            int height = outputImage.rows();

            int clawCenterX = (int) (CLAW_CENTER_X_CM / PIXELS_TO_CM_RATIO + (double) width / 2);
            int clawCenterY = (int) (height - (CLAW_CENTER_Y_CM + CLAW_OFFSET_FROM_CAMERA_CM) / PIXELS_TO_CM_RATIO);
            Imgproc.drawMarker(outputImage, new Point(clawCenterX, clawCenterY),
                    new Scalar(0, 255, 255), Imgproc.MARKER_CROSS, 20, 2);

            DetectedCube target = null;
            double minDistance = Double.MAX_VALUE;

            for (DetectedCube cube : detectedCubes) {
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


        private double[] calculateWorldCoordinates(int x, int y, int imgWidth, int imgHeight) {
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