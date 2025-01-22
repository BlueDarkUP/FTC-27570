package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ChamberDrive_4specimens",group = "Competition")
public class Chamber_Drive_4Specimens extends LinearOpMode {

    // public double CM_TO_INCH(double cm){
    //     return cm*0.39370;
    //}
    private DcMotorEx Left_Hanging_Motor, Right_Hanging_Motor;
    private static final  int LIFT_UP_POSITION = 1300;
    private static final  int LIFT_DOWN_POSITION = 500;
    private static final int LIFT_RESET_POSITION = 0;

    private static final int WCXS= 6;
    private Servo backgrap,frame,forward_slide, arm_forward, claw_shu, forward_claw, claw_heng;

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
        backgrap = hardwareMap.get(Servo.class, "backgrap");
        frame = hardwareMap.get(Servo.class, "frame");
        forward_slide = hardwareMap.get(Servo.class, "forward_slide");
        arm_forward = hardwareMap.get(Servo.class, "arm_forward");
        claw_shu = hardwareMap.get(Servo.class, "claw_shu");
        forward_claw = hardwareMap.get(Servo.class, "forward_claw");
        claw_heng = hardwareMap.get(Servo.class, "claw_heng");
        claw_heng.setPosition(0.55);
        frame.setPosition(0.7);
        backgrap.setPosition(0.5);
        forward_slide.setPosition(1);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(0.5);
        telemetry.addData("LLM",Left_Hanging_Motor);
        telemetry.addData("RLM",Right_Hanging_Motor);
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-62.5,-10.5,0))
                        .setTangent(0)
                        //Lift up first

                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .splineToConstantHeading(new Vector2d(-33, -10.5), 0)
                        //First Chamber set
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.17)
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        .waitSeconds(0.1)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        //Reset lift
                        //Go to push 2 Sample

                        .setTangent(180)
                        .splineToSplineHeading(new Pose2d(-46, -22, Math.PI/2), -Math.PI/2)
                        .splineToSplineHeading(new Pose2d(-38, -34, Math.PI), 0)
                        .splineToConstantHeading(new Vector2d(-20, -35), 0)
                        .splineToConstantHeading(new Vector2d(-8, -40), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-20, -37), Math.PI)
                        .splineToConstantHeading(new Vector2d(-45, -45), Math.PI)
                        //1

                        .waitSeconds(0.05)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-20, -41), 0)
                        .splineToConstantHeading(new Vector2d(-6, -54), -Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-20, -52), Math.PI)
                        .splineToConstantHeading(new Vector2d(-45, -55), Math.PI)
                        .setTangent(0)
                        //ahead is about 2 sample
                        .splineToConstantHeading(new Vector2d(-50, -37), Math.PI)
                        //Wait for human player to set the sample
                        .splineToConstantHeading(new Vector2d(-61.5, -35), Math.PI)
                        //Get Sample
                        .stopAndAdd(new PatientServoAction(backgrap,0.5))
                        .waitSeconds(0.2)
                        //Lift up first
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(-31, -6, 0), 0)
                        .waitSeconds(0.1)
                        //Set Sample 2
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.17)
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        .waitSeconds(0.13)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(180)

                        .splineToSplineHeading(new Pose2d(-53,-37,Math.PI),Math.PI)
                        .splineToConstantHeading(new Vector2d(-61.5,-37),0)
                        .stopAndAdd(new PatientServoAction(backgrap,0.5))
                        .waitSeconds(0.2)

                        //Lift up first
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .splineToSplineHeading(new Pose2d(-31,-3,0),0)
                        .waitSeconds(0.1)
                        //set sample
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.17)
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        .waitSeconds(0.13)
                        .setTangent(180)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .splineToSplineHeading(new Pose2d(-53,-37,Math.PI),Math.PI)
                        .splineToConstantHeading(new Vector2d(-61.5,-37),0)
                        .stopAndAdd(new PatientServoAction(backgrap,0.5))
                        .waitSeconds(0.2)

                        //Lift up first
                        .setTangent(0)
                        .stopAndAdd(new ServoAction(forward_slide,0.4))
                        .stopAndAdd(new ServoAction(arm_forward,0.3))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1400))
                        .splineToSplineHeading(new Pose2d(-32.5,0,0),0)

                        .waitSeconds(0.1)
                        //set sample
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.17)
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        .waitSeconds(0.13)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(180)
                        .splineToSplineHeading(new Pose2d(-47,-27,Math.PI /4),-Math.PI*((double) 3 /4))
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
                Left_Hanging_Motor.setPower(1);
                Right_Hanging_Motor.setPower(1);
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
            Left_Hanging_Motor.setPower(1);
            Right_Hanging_Motor.setPower(1);
            return false;
        }
    }
    public class ServoAction implements Action {
        Servo FrontSlide = null;
        double position = 0;
        boolean hasInitialized = false;

        public ServoAction(Servo s, double p) {
            this.FrontSlide = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FrontSlide.setPosition(position);
            return false;
        }
    }
}
