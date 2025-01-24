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
        while (opModeIsActive()) {
            // 检测 gamePad1 的圆形按钮是否被按下
            if (gamepad1.circle) {
                // 记录当前机器人的位置
                Pose2d recordPose = drive.pose;
                // 通过遥测数据输出记录的机器人位置
                telemetry.addData("Recorded Pose Basket", "X: %.2f, Y: %.2f, Heading: %.2f",
                        recordPose.position.x, recordPose.position.y, Math.toDegrees(recordPose.heading.toDouble()));
                telemetry.addLine("Processing frame..."); // 添加一行消息
                telemetry.update(); // 更新遥测数据

                // 等待圆形按钮松开，同时 OpMode 仍处于活动状态
                while (gamepad1.circle && opModeIsActive()) {
                    idle(); // 空闲
                }

                // 使用 Road Runner 的 ActionBuilder 构建一个动作，将机器人移动到指定位置
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose) // 从当前位置开始构建动作
                                .splineToConstantHeading(
                                        new Vector2d(recordPose.position.x + result.moveForward * 0.39370,
                                                recordPose.position.y + result.moveSideways * 0.39370),
                                        0.0) // 移动到指定位置，并保持方向不变
                                .build()
                );
                telemetry.addLine("Moved to pose"); // 输出成功移动的消息
                telemetry.update(); // 更新遥测数据
            }
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
                telemetry.addData("Claw Angle", result.clawAngle + "°");
                telemetry.update(); // 更新遥测数据
            }

            // 返回原始的图像帧
            return input;
        }
    }
}