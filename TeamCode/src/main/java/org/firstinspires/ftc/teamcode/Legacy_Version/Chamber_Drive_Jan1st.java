package org.firstinspires.ftc.teamcode.Legacy_Version;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Autonomous(name = "First_Auto_ChamberDrive_BY27570_Jan1st",group = "old")
public class Chamber_Drive_Jan1st extends LinearOpMode {

    // public double CM_TO_INCH(double cm){
    //     return cm*0.39370;
    //}
    private DcMotorEx Left_Hanging_Motor, Right_Hanging_Motor;
    private static final  int LIFT_UP_POSITION = 1650;
    private static final  int LIFT_DOWN_POSITION = 800;
    private static final int LIFT_RESET_POSITION = 0;

    private static final int WCXS= 6;
    //private Servo backgrap;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive Drive = new MecanumDrive(hardwareMap, new Pose2d(-62.5,-9.5,0));
        Left_Hanging_Motor = hardwareMap.get(DcMotorEx.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotorEx.class, "RightHangingMotor");
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backgrap = hardwareMap.get(Servo.class, "backgrap");

        //backgrap.setPosition(GRAB_CLOSE);
        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-62.5,-9.5,0))
                        .setTangent(0)
                        //Lift up first
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .splineToConstantHeading(new Vector2d(-34.5, -9.5), Math.PI)
                        //First Chamber set
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_DOWN_POSITION))
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_OPEN))
                        //Reset lift
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        //Go to push 3 Sample
                        .setTangent(180)
                        .splineToSplineHeading(new Pose2d(-46, -22, Math.PI/2), -Math.PI/2)
                        .splineToSplineHeading(new Pose2d(-38, -34, Math.PI), 0)
                        .splineToConstantHeading(new Vector2d(-25, -37), 0)
                        .splineToConstantHeading(new Vector2d(-13, -43), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-20, -48), Math.PI)
                        .splineToConstantHeading(new Vector2d(-48, -48), Math.PI)
                        //1
                        .splineToConstantHeading(new Vector2d(-20, -42), 0)
                        .splineToConstantHeading(new Vector2d(-13, -50.5), -Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-20, -58), Math.PI)
                        .splineToConstantHeading(new Vector2d(-45, -58), Math.PI)
                        //ahead is about 2 sample


                        .splineToConstantHeading(new Vector2d(-50, -37), Math.PI)
                        //Wait for human player to set the sample
                        .waitSeconds(0.3)
                        .splineToConstantHeading(new Vector2d(-61, -37), Math.PI)
                        //Get Sample
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_CLOSE))
                        //Lift up first
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .splineToSplineHeading(new Pose2d(-34.5, -6, 0), 0)
                        //Todo:Set Sample 2
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_DOWN_POSITION))
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_OPEN))
                        .setTangent(180)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        .splineToSplineHeading(new Pose2d(-53,-37,Math.PI),Math.PI)
                        .waitSeconds(0.3)
                        .splineToConstantHeading(new Vector2d(-61,-37),0)
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_CLOSE))

                        //Lift up first
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .splineToSplineHeading(new Pose2d(-34.5,-3,0),0)
                        //set sample
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_DOWN_POSITION))
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_OPEN))
                        .setTangent(180)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        .splineToSplineHeading(new Pose2d(-53,-37,Math.PI),Math.PI)
                        .waitSeconds(0.3)
                        .splineToConstantHeading(new Vector2d(-61,-37),0)
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_CLOSE))

                        //3
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .splineToSplineHeading(new Pose2d(-34.5,-0,0),0)
                        //set sample
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_DOWN_POSITION))
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_OPEN))
                        .setTangent(180)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        .splineToConstantHeading(new Vector2d(-58,-27),-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-60,-60),-Math.PI/2)
                        //.waitSeconds(0.3)
                        //.splineToConstantHeading(new Vector2d(60,37),0)
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_CLOSE))
                        .build());
    }
    public class PatientServoAction implements Action {
        Servo FrontSlide = null;
        double position = 0;
        boolean hasInitialized = false;
        public PatientServoAction(Servo s,double p){
            this.FrontSlide = s;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!hasInitialized){
                FrontSlide.setPosition(position);
                hasInitialized = true;
            }
            return false;
        }
    }
    public class PatientMotorAction implements Action {
        DcMotorEx Left_Hanging_Motor = null;
        DcMotorEx Right_Hanging_Motor = null;
        int position = 0;

        boolean hasInitialized = false;
        public PatientMotorAction(DcMotorEx M1,DcMotorEx M2,int p){
            this.Left_Hanging_Motor = M1;
            this.Right_Hanging_Motor = M2;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!hasInitialized){
                Left_Hanging_Motor.setTargetPosition(position);
                Right_Hanging_Motor.setTargetPosition(position);
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Hanging_Motor.setPower(0.8);
                Right_Hanging_Motor.setPower(0.8);
                hasInitialized = true;
            }
            if(Left_Hanging_Motor.isBusy() && Right_Hanging_Motor.isBusy()){
                return true;
            }
            return false;
        }
    }
    public class MotorAction implements Action {
        DcMotorEx Left_Hanging_Motor = null;
        DcMotorEx Right_Hanging_Motor = null;
        int position = 0;

        public MotorAction(DcMotorEx M1,DcMotorEx M2,int p){
            this.Left_Hanging_Motor = M1;
            this.Right_Hanging_Motor = M2;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Left_Hanging_Motor.setTargetPosition(position);
            Right_Hanging_Motor.setTargetPosition(position);
            Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Hanging_Motor.setPower(0.8);
            Right_Hanging_Motor.setPower(0.8);
            return false;
        }
    }
}
