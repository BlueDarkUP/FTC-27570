package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CubeDetection;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.CubeDetection.CubeDetectionResult;

@TeleOp(name = "CubeDetectionTest", group = "TeleOp")
public class Main extends LinearOpMode {

    private OpenCvWebcam webcam; // 使用 OpenCvWebcam
    private CubeDetectionResult result;
    private Gamepad gamepad1;

    @Override
    public void runOpMode() {
        // 1. 初始化摄像头
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // 修改为你的摄像头名称
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); // 使用 OpenCvWebcam

        // 2. 设置管道
        webcam.setPipeline(new CubeDetectionPipeline());

        // 3. 打开摄像头
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                System.out.println("Info: Camera successfully opened");

                // 4. 启动摄像头预览
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                // 5. 初始化游戏手柄
                gamepad1 = gamepad1;

                // 6. 等待 start
                waitForStart();

                if (isStopRequested()) return; // Exit if stop is requested

                // 7. 检测并移动
                while (opModeIsActive()) {
                    // 8. 监听手柄圆形按钮
                    if (gamepad1.circle) {
                        // Process the frame using the pipeline
                        // The pipeline will handle frame processing asynchronously
                        telemetry.addLine("Processing frame...");
                        telemetry.update();

                        //等待按键松开，防止多次触发
                        while (gamepad1.circle && opModeIsActive()) {
                            idle();
                        }
                    }
                }
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("Error: Could not open camera. Error Code: " + errorCode);
            }
        });
    }

    // Custom pipeline for cube detection
    class CubeDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Process the frame here
            String allianceColor = "red"; // Set your alliance color
            double clawCenterX = 0; // Set your claw center X
            double clawCenterY = 0; // Set your claw center Y

            result = CubeDetection.detectAndMove(input, allianceColor, clawCenterX, clawCenterY);
            if (result != null) {
                telemetry.addLine("-------------------- Final Results --------------------");
                telemetry.addData("Result", result.toString());
                telemetry.update();
            }

            return input; // Return the processed frame
        }
    }
}