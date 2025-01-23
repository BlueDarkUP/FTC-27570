package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class CubeDetection {

    // --- 常量定义 ---
    private static final double CLAW_OFFSET_FROM_CAMERA_CM = 5;
    private static final double PIXELS_TO_CM_RATIO = 0.01;
    private static final int MIN_REGION_WIDTH = 0;
    private static final int MIN_REGION_HEIGHT = 0;
    private static final double EXPECTED_ASPECT_RATIO_SHORT_LONG = 3.0 / 7.0;
    private static final double ASPECT_RATIO_TOLERANCE = 0.3;


    public static class CubeDetectionResult {
        public double moveForward;
        public double moveSideways;
        public double clawAngle;

        public CubeDetectionResult(double moveForward, double moveSideways, double clawAngle) {
            this.moveForward = moveForward;
            this.moveSideways = moveSideways;
            this.clawAngle = clawAngle;
        }

        public String toString() {
            return "Move Forward: " + String.format("%.1f", moveForward) + " cm\n" +
                    "Move Sideways: " + String.format("%.1f", moveSideways) + " cm\n" +
                    "Claw Angle: " + String.format("%.1f", clawAngle) + "°";
        }
    }

    private static class DetectedCube {
        public String color;
        public int centerXImagePx;
        public int centerYImagePx;
        public double centerXCm;
        public double centerYCm;
        public double angleDegrees;
        public Rect boundingBox;
        public double aspectRatio;
        public double distanceToClawCm;

        public DetectedCube(String color, int centerXImagePx, int centerYImagePx, double centerXCm, double centerYCm, double angleDegrees, Rect boundingBox, double aspectRatio) {
            this.color = color;
            this.centerXImagePx = centerXImagePx;
            this.centerYImagePx = centerYImagePx    ;
            this.centerXCm = centerXCm;
            this.centerYCm = centerYCm;
            this.angleDegrees = angleDegrees;
            this.boundingBox = boundingBox;
            this.aspectRatio = aspectRatio;
            this.distanceToClawCm = -1;
        }
    }


    public static CubeDetectionResult detectAndMove(Mat input, String allianceColor, double clawCenterXCm, double clawCenterYCm) {
        if(input == null || input.empty()) {
            System.out.println("Error: Input Mat is null or empty");
            return null;
        }
        if(allianceColor == null || allianceColor.isEmpty()) {
            System.out.println("Error: Alliance color is null or empty");
            return null;
        }

        Mat outputImage = new Mat();
        input.copyTo(outputImage);
        Mat hsvImage = new Mat();
        Mat labImage = new Mat();

        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(input, labImage, Imgproc.COLOR_BGR2Lab);
        List<DetectedCube> detectedCubes = new ArrayList<>();


        detectCubes(outputImage, hsvImage, allianceColor, detectedCubes);
        CubeDetectionResult result = performClawMovement(outputImage, detectedCubes, clawCenterXCm, clawCenterYCm, allianceColor);
        printDetectionResults(detectedCubes, clawCenterXCm, clawCenterYCm, outputImage.height());

        hsvImage.release();
        labImage.release();
        outputImage.release();
        return result;
    }

    private static void detectCubes(Mat outputImage, Mat hsvImage, String allianceColor, List<DetectedCube> detectedCubes) {
        String[] colorNames = {"red", "blue", "yellow"};
        Scalar[][] colorRangesHsv = new Scalar[3][2];

        colorRangesHsv[0][0] = new Scalar(170, 135, 220);
        colorRangesHsv[0][1] = new Scalar(180, 255, 255);
        colorRangesHsv[1][0] = new Scalar(100, 100, 210);
        colorRangesHsv[1][1] = new Scalar(130, 255, 255);
        colorRangesHsv[2][0] = new Scalar(20, 165, 248);
        colorRangesHsv[2][1] = new Scalar(40, 255, 255);

        Mat mask = new Mat();
        Mat currentMask = new Mat();

        for (int i = 0; i < colorNames.length; i++) {
            String colorName = colorNames[i];
            mask.setTo(new Scalar(0));

            if (colorName.equals("red")) {
                Core.inRange(hsvImage, colorRangesHsv[0][0], colorRangesHsv[0][1], currentMask);
                Core.bitwise_or(mask, currentMask, mask);
                Core.inRange(hsvImage, new Scalar(0, 135, 220), new Scalar(10, 255, 255), currentMask);
                Core.bitwise_or(mask, currentMask, mask);
            } else {
                Core.inRange(hsvImage, colorRangesHsv[i][0], colorRangesHsv[i][1], mask);
            }

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area > 500) {
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));
                    double angleRect = rect.angle;
                    Size sizeRect = rect.size;
                    double wRect = sizeRect.width;
                    double hRect = sizeRect.height;

                    Rect boundingRect = Imgproc.boundingRect(cnt);
                    int x = boundingRect.x;
                    int y = boundingRect.y;
                    int w = boundingRect.width;
                    int h = boundingRect.height;

                    if (w < MIN_REGION_WIDTH || h < MIN_REGION_HEIGHT) {
                        continue;
                    }

                    int centerX = x + w / 2;
                    int centerY = y + h / 2;
                    double aspectRatioDetected = 0.0;
                    double angleDegrees = 0.0;

                    if (wRect > 0 && hRect > 0) {
                        if (wRect > hRect) {
                            aspectRatioDetected = hRect / wRect;
                            angleDegrees = angleRect;
                        } else {
                            aspectRatioDetected = wRect / hRect;
                            angleDegrees = angleRect + 90;
                        }
                        angleDegrees = angleDegrees % 180;
                    }

                    if (aspectRatioDetected >= (EXPECTED_ASPECT_RATIO_SHORT_LONG - ASPECT_RATIO_TOLERANCE) &&
                            aspectRatioDetected <= (EXPECTED_ASPECT_RATIO_SHORT_LONG + ASPECT_RATIO_TOLERANCE)) {
                        DetectedCube cube = new DetectedCube(colorName, centerX, centerY,
                                calculateWorldCoordinates(centerX, centerY, outputImage.height()).x,
                                calculateWorldCoordinates(centerX, centerY, outputImage.height()).y,
                                angleDegrees, boundingRect, aspectRatioDetected);
                        detectedCubes.add(cube);

                        if (colorName.equalsIgnoreCase(allianceColor)) {
                            System.out.println("Detected Alliance Color Cube: " + colorName.toUpperCase());
                        }
                    }
                }
            }
            contours.forEach(Mat::release);
            hierarchy.release();
            currentMask.release();
        }
        mask.release();
    }

    private static CubeDetectionResult performClawMovement(Mat outputImage, List<DetectedCube> detectedCubes, double clawCenterXCm, double clawCenterYCm, String allianceColor) {
        int height = outputImage.height();

        DetectedCube closestCube = null;
        List<DetectedCube> validCubesAlliance = new ArrayList<>();
        List<DetectedCube> validCubesNeutral = new ArrayList<>();
        double moveForward = 0;
        double moveSideways = 0;
        double clawAngle = 0;

        if (!detectedCubes.isEmpty()) {
            for (DetectedCube cube : detectedCubes) {
                if (cube.color.equals("yellow")) {
                    validCubesNeutral.add(cube);
                } else if (cube.color.equalsIgnoreCase(allianceColor)) {
                    validCubesAlliance.add(cube);
                }
            }

            if (!validCubesAlliance.isEmpty()) {
                for (DetectedCube cube : validCubesAlliance) {
                    cube.distanceToClawCm = Math.sqrt(Math.pow(cube.centerXCm - clawCenterXCm, 2) + Math.pow(cube.centerYCm - clawCenterYCm, 2));
                }
                validCubesAlliance.sort(Comparator.comparingDouble(cube -> cube.distanceToClawCm));
                closestCube = validCubesAlliance.get(0);
            } else if (!validCubesNeutral.isEmpty()) {
                for (DetectedCube cube : validCubesNeutral) {
                    cube.distanceToClawCm = Math.sqrt(Math.pow(cube.centerXCm - clawCenterXCm, 2) + Math.pow(cube.centerYCm - clawCenterYCm, 2));
                }
                validCubesNeutral.sort(Comparator.comparingDouble(cube -> cube.distanceToClawCm));
                closestCube = validCubesNeutral.get(0);
            }

            if (closestCube != null) {
                moveForward = closestCube.centerYCm - clawCenterYCm;
                moveSideways = closestCube.centerXCm - clawCenterXCm;
                clawAngle = -closestCube.angleDegrees;

                System.out.println("\n" + "-------------------- 运动指令 --------------------");
                System.out.println("目标颜色: " + closestCube.color.toUpperCase());
                System.out.println("前进距离: " + String.format("%.1f", moveForward) + " cm");
                System.out.println("横向移动: " + String.format("%.1f", moveSideways) + " cm");
                System.out.println("爪子旋转: " + String.format("%.1f", clawAngle) + "°");
                System.out.println("直线距离: " + String.format("%.1f", closestCube.distanceToClawCm) + " cm");
            }
        }
        return new CubeDetectionResult(moveForward, moveSideways, clawAngle);
    }

    private static void printDetectionResults(List<DetectedCube> detectedCubes, double clawCenterXCm, double clawCenterYCm, int imageHeight) {
        System.out.println("\n" + "-------------------- 检测结果 (HSV, 小框过滤, 宽高比角度) --------------------");
        if (detectedCubes.isEmpty()) {
            System.out.println("未检测到任何长方体");
        } else {
            for (int i = 0; i < detectedCubes.size(); i++) {
                DetectedCube cube = detectedCubes.get(i);
                System.out.println("\n长方体 " + (i + 1) + ":");
                System.out.println("  颜色: " + cube.color.toUpperCase());
                System.out.println("  图像坐标: (" + cube.centerXImagePx + ", " + cube.centerYImagePx + ")px");
                System.out.println("  世界坐标: (" + String.format("%.1f", cube.centerXCm) + "cm, " + String.format("%.1f", cube.centerYCm) + "cm)");
                System.out.println("  旋转角度: " + String.format("%.1f", cube.angleDegrees) + "°");
                System.out.println("  宽高比 (短/长): " + String.format("%.2f", cube.aspectRatio));
            }
        }
        int width = 0;
        if(imageHeight != 0) {
            width = (int)(2.0 * imageHeight / 3.0);
        }
        int clawCenterXPx = (int) (clawCenterXCm / PIXELS_TO_CM_RATIO + width / 2.0);
        int clawCenterYPx = (int) (imageHeight - (clawCenterYCm + CLAW_OFFSET_FROM_CAMERA_CM) / PIXELS_TO_CM_RATIO);

        System.out.println("\n" + "-------------------- 当前爪子位置 --------------------");
        System.out.println("爪子图像坐标: (" + clawCenterXPx + ", " + clawCenterYPx + ")px");
        System.out.println("爪子世界坐标: (" + String.format("%.1f", calculateWorldCoordinates(clawCenterXPx, clawCenterYPx, imageHeight).x) + "cm, " + String.format("%.1f", calculateWorldCoordinates(clawCenterXPx, clawCenterYPx, imageHeight).y) + "cm)");
    }

    private static Point calculateWorldCoordinates(int centerXPx, int centerYPx, int imageHeight) {
        double xCm = (centerXPx - imageHeight / 2.0) * PIXELS_TO_CM_RATIO;
        double yCm = ((imageHeight - centerYPx) * PIXELS_TO_CM_RATIO) - CLAW_OFFSET_FROM_CAMERA_CM;
        return new Point(xCm, yCm);
    }
}