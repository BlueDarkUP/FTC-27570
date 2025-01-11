package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Manual with Return To Position", group = "manual")
public class ManualWithReturnToPosition extends LinearOpMode {

    private MecanumDrive drive;
    private Pose2d targetPose = null;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            // Manual driving with joystick
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            // Record target position with left bumper
            if (gamepad1.left_bumper && targetPose == null) {
                targetPose = drive.getPoseEstimate();
                telemetry.addData("Target Pose Recorded:", targetPose);
                telemetry.update();
            }

            // Return to target position with right bumper
            if (gamepad1.right_bumper && targetPose != null) {
                Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(targetPose.vec(), targetPose.getHeading())
                        .build();

                drive.followTrajectory(trajectory);

                // Reset targetPose after reaching it
                targetPose = null;
                telemetry.addData("Returned to Target", "");
                telemetry.update();

                // Wait until trajectory finishes (optional but recommended)
                while(opModeIsActive() && drive.isBusy()){
                    drive.update();
                }
            }


            // Allow for smooth driving by constantly updating RoadRunner
            drive.update();


            telemetry.addData("Current Pose", drive.getPoseEstimate());
            telemetry.update();
        }
    }
}