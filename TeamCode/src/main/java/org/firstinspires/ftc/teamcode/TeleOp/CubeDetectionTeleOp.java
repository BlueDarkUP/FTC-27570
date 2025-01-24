package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CubeDetection;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.CubeDetection.CubeDetectionResult;

// 标记这个类为一个 TeleOp 模式，并设置名称和组别
@TeleOp(name = "CubeDetectionTest", group = "TeleOp")
public class CubeDetectionTeleOp extends LinearOpMode {
    // 声明一个 OpenCvWebcam 对象，用于摄像头
    private OpenCvWebcam webcam;
    // 声明一个 CubeDetectionResult 对象，用于存储图像处理的结果
    private CubeDetectionResult result;

    // OpMode 的主运行方法
    @Override
    public void runOpMode() {
        // 1. 初始化摄像头
        // 获取摄像头监控视图的 ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // 获取名为 "Webcam 1" 的摄像头硬件
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // 使用 OpenCvCameraFactory 创建一个 OpenCvWebcam 对象
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // 设置摄像头的处理管道为自定义的 CubeDetectionPipeline
        webcam.setPipeline(new CubeDetectionPipeline());

        // 异步打开摄像头设备
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // 当摄像头成功打开时调用
            @Override
            public void onOpened() {
                System.out.println("Info: Camera successfully opened");
                // 开始摄像头图像流，设置分辨率为 640x480，并旋转为竖直方向
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            // 当摄像头打开失败时调用
            @Override
            public void onError(int errorCode) {
                System.out.println("Error: Could not open camera. Error Code: " + errorCode);
            }
        });

        // 创建一个 MecanumDrive 对象，用于控制机器人运动
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // 等待开始按钮按下
        waitForStart();

        // 如果收到停止请求，则退出
        if (isStopRequested()) return;

        // 主循环，当 OpMode 处于活动状态时持续运行
        // 修改主循环部分
        while (opModeIsActive()) {
            if (gamepad1.circle) {
                CubeDetectionResult currentResult = this.result; // 获取当前结果快照
                if (currentResult == null) {
                    telemetry.addData("Error", "No detection result yet.");
                    telemetry.update();
                    continue;
                }

                Pose2d recordPose = drive.pose;
                // 使用currentResult代替result
                double forwardInches = currentResult.moveForward * 0.39370;
                double sidewaysInches = currentResult.moveSideways * 0.39370;

                Actions.runBlocking(
                        drive.actionBuilder(recordPose)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                recordPose.position.x + forwardInches,
                                                recordPose.position.y + sidewaysInches
                                        ), 0.0)
                                .build()
                );

                while (gamepad1.circle && opModeIsActive()) { idle(); }
            }
            telemetry.update(); // 主线程统一更新遥测
        }
    }

    // 自定义的 OpenCvPipeline 类，用于图像处理
    class CubeDetectionPipeline extends OpenCvPipeline {
        // 重写 processFrame 方法，处理每一帧图像
        @Override
        public Mat processFrame(Mat input) {
            // 设置联盟颜色，这里默认为红色
            String allianceColor = "red";
            // 爪子的中心坐标，这里默认为 0
            double clawCenterX = 0;
            double clawCenterY = 0;

            // 使用 CubeDetection 类检测立方体并计算移动距离
            result = CubeDetection.detectAndMove(input, allianceColor, clawCenterX, clawCenterY);

            // 如果结果不为空，则输出结果
            if (result != null) {
                telemetry.addLine("-------------------- Final Results --------------------");
                telemetry.addData("Move Forward", result.moveForward + " cm");
                telemetry.addData("Move Sideways", result.moveSideways + " cm");
                telemetry.addData("Claw Angle", result.clawAngle     + "°");
                telemetry.update(); // 更新遥测数据
            }

            // 返回原始的图像帧
            return input;
        }
    }
}