package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Record and Return", group = "TeleOp")
public class RecordAndReturn extends LinearOpMode {
    // 底盘驱动对象
    private MecanumDrive drive;

    // 记录的位置和状态
    private Pose2d recordedPose = null;

    // 防抖计时器
    private ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 初始化底盘驱动
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // 等待比赛开始
        waitForStart();

        while (opModeIsActive()) {
            // 更新机器人的位置和朝向
            drive.updatePoseEstimate();

            // 通过摇杆控制底盘运动
            driveControl();

            // 处理 left bumper 按下事件
            if (gamepad1.left_bumper && debounceTimer.seconds() > 0.5) { // 防抖延时 0.5 秒
                debounceTimer.reset();

                if (recordedPose == null) {
                    // 第一次按下 left bumper：记录当前位置和状态
                    recordedPose = drive.pose;
                    telemetry.addData("Recorded Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                            recordedPose.position.x, recordedPose.position.y, Math.toDegrees(recordedPose.heading.toDouble()));
                    telemetry.update();
                } else {
                    // 第二次按下 left bumper：规划路径回到记录的位置
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToSplineHeading(recordedPose, 0) // 使用 splineToSplineHeading
                                    .build()
                    );

                    // 重置记录的位置
                    recordedPose = null;
                }
            }

            // 更新 Telemetry
            telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                    drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
    // 通过摇杆控制底盘运动
    private void driveControl() {
        // 获取摇杆输入
        double y = gamepad1.left_stick_y; // 前后移动
        double x = -gamepad1.left_stick_x; // 左右移动

        // 使用左右扳机键控制旋转
        double leftTrigger = gamepad1.left_trigger; // 左扳机键 (0 到 1)
        double rightTrigger = gamepad1.right_trigger; // 右扳机键 (0 到 1)
        double rx = rightTrigger - leftTrigger; // 旋转控制：右扳机键增加旋转，左扳机键减少旋转

        // 计算机器人运动功率
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // 设置电机功率
        drive.leftFront.setPower(frontLeftPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.rightBack.setPower(backRightPower);
    }
}