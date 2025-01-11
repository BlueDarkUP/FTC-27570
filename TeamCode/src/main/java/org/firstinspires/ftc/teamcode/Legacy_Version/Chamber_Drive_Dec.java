package org.firstinspires.ftc.teamcode.Legacy_Version;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Autonomous(name = "First_Auto_ChamberDrive_BY27570_Dec",group = "old")
public class Chamber_Drive_Dec extends LinearOpMode {

    // public double CM_TO_INCH(double cm){
    //     return cm*0.39370;
    //}
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive Drive = new MecanumDrive(hardwareMap, new Pose2d(-62.5,-9.5,0));
        //Servo FrontSlide = hardwareMap.servo.get("TTT");

        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-62.5,-9.5,0))
                        .lineToX(-35.5)
                        //Todo：First Chamber Sample

                        .lineToX(-45)
                        .setTangent(180)
                        .splineToLinearHeading(new Pose2d(-27, -40, Math.PI), 0)
                        .splineToConstantHeading(new Vector2d(-14, -43.5), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-19, -47), Math.PI)
                        .splineToConstantHeading(new Vector2d(-50, -48), 0)
                        .waitSeconds(0.1)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-12, -48), 0)
                        .splineToConstantHeading(new Vector2d(-12, -55), Math.PI)
                        .splineToConstantHeading(new Vector2d(-48, -55), 0)
                        .waitSeconds(0.1)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-12, -58), 0)
                        .splineToConstantHeading(new Vector2d(-12, -62), Math.PI)
                        .splineToConstantHeading(new Vector2d(-48, -62), 0)
                        .splineToConstantHeading(new Vector2d(-52, -36), Math.PI)
                        .waitSeconds(1.5)
                        .setTangent(180)
                        .splineToConstantHeading(new Vector2d(-60, -36), 0)
                        //Todo: Get First Sample

                        .splineToLinearHeading(new Pose2d(-35, -7, 0), 0)
                        //Todo: Set Sample

                        .setTangent(180)
                        .splineToLinearHeading(new Pose2d(-24, -36, 180), 0)

                        .build());
    }
    public class ServoAction implements Action {
        Servo FrontSlide = null;
        double position = 0;
        public ServoAction(Servo s,double p){
            this.FrontSlide = s;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FrontSlide.setPosition(position);
            return false;
        }
    }
    public class PatientServoAction implements Action {
        Servo FrontSlide = null;
        double position = 0;
        ElapsedTime timer;

        public PatientServoAction(Servo s,double p){
            this.FrontSlide = s;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null){
                timer = new ElapsedTime();
                FrontSlide.setPosition(position);
            }

            return false;
        }
    }
}
