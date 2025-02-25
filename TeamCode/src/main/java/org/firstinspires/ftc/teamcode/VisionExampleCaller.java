package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@TeleOp(name = "VisionAPI 调用范例 - Static")
public class VisionExampleCaller extends LinearOpMode {

    private WebcamName webcamName;

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo armForwardServo = hardwareMap.get(Servo.class, "arm_forward");
        Servo clawShuServo = hardwareMap.get(Servo.class, "claw_shu");
        Servo clawHengServo = hardwareMap.get(Servo.class, "claw_heng");
        Servo forwardClawServo = hardwareMap.get(Servo.class, "forward_claw");
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        if (webcamName == null) {
            telemetry.addData("摄像头错误", "找不到名为 'Webcam' 的摄像头硬件配置.");
            telemetry.update();
            return;
        }

        if (!VisionAPI.initialize(webcamName, telemetry)) { // Static initialization call
            telemetry.addLine("VisionAPI 初始化失败，请检查配置。");
            telemetry.update();
            while (opModeInInit()) {
                idle();
            }
            return;
        }
        telemetry.addLine("VisionAPI 初始化完成，等待开始...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 接近
        VisionAPI.ApproachData approachData = VisionAPI.getApproachData();
        Vector2d targetPosition = new Vector2d(
                drive.pose.position.x + approachData.moveForwardcm * 0.39370,
                drive.pose.position.y - approachData.moveSidewaysCm * 0.39370
        );
        Action approachMovement = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(targetPosition, 0)
                .build();
        Actions.runBlocking(approachMovement);

        // 移动到抓取位置
        VisionAPI.GrabData grabData = VisionAPI.getGrabData();
        telemetry.addData("抓取前进距离 (cm)", grabData.moveForwardCm);
        telemetry.addData("抓取横向移动 (cm)", grabData.moveSidewaysCm);
        telemetry.addData("爪子舵机位置 (0-1)", grabData.servoPosition);
        telemetry.update();
        sleep(2000); // 实际的roadrunner动作
        Action visionBasedMovement = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(
                        new Vector2d(drive.pose.position.x + grabData.moveSidewaysCm * 0.39370, drive.pose.position.y - grabData.moveSidewaysCm * 0.39370), 0
                )
                .stopAndAdd(new SingleStickWithArm.ServoAction(armForwardServo, 0.4))
                .stopAndAdd(new SingleStickWithArm.ServoAction(clawHengServo, grabData.servoPosition))
                .stopAndAdd(new SingleStickWithArm.ServoAction(clawShuServo, 1))
                .build();

        Actions.runBlocking(visionBasedMovement);

        VisionAPI.stopCamera();
        telemetry.addLine("程序结束，摄像头已停止。");
        telemetry.update();
        while (!isStopRequested()) {
            idle();
        }
    }
}