package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.OPT_TuningRunner;

@Autonomous(name = "FuckingBest_5_SpecimensWithFuckingPark",group = "Competition")
public class ChamberDrive5Park extends LinearOpMode {

    // public double CM_TO_INCH(double cm){
    //     return cm*0.39370;
    //}
    private DcMotorEx Left_Hanging_Motor, Right_Hanging_Motor,BigArm;
    private static final  int LIFT_UP_POSITION = 610;
    private static final int BIG_ARM_SET_POSITION = 387;
    private static final double BACK_ARM_SET_POSITION = 0.82;
    private static final double BACK_ARM_RESET_POSITION = 0.17;
    private static final int LIMIT_VEL= 28;
    private Servo FuckingArm,backgrap,forward_claw,claw_shu,claw_heng,arm_forward,forward_slide,forward_slide2;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive Drive = new MecanumDrive(hardwareMap, new Pose2d(-62.5,0,Math.PI));
        Left_Hanging_Motor = hardwareMap.get(DcMotorEx.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotorEx.class, "RightHangingMotor");
        forward_claw = hardwareMap.get(Servo.class,"forward_claw");
        forward_slide = hardwareMap.get(Servo.class,"forward_slide");
        arm_forward = hardwareMap.get(Servo.class,"arm_forward");
        claw_heng = hardwareMap.get(Servo.class,"claw_heng");
        claw_shu = hardwareMap.get(Servo.class,"claw_shu");
        BigArm = hardwareMap.get(DcMotorEx.class,"big_arm");
        FuckingArm = hardwareMap.get(Servo.class,"back_arm");
        backgrap = hardwareMap.get(Servo.class,"backgrap");
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BigArm.setDirection(DcMotorSimple.Direction.REVERSE);

        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BigArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BigArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BigArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("LLM",Left_Hanging_Motor);
        telemetry.addData("RLM",Right_Hanging_Motor);
        telemetry.update();

        backgrap.setPosition(0.6);
        FuckingArm.setPosition(0.17);
        claw_heng.setPosition(0.55);
        forward_claw.setPosition(0.88);
        forward_slide.setPosition(0);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(0);
        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-62.5,0,Math.PI))
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .waitSeconds(0.1)
                        .stopAndAdd(new ArmMotorAction(BigArm,false,BIG_ARM_SET_POSITION))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_SET_POSITION))
                        .splineToConstantHeading(new Vector2d(-33.1, 0), 0)
                        .stopAndAdd(new ServoAction(backgrap,0))
                        .waitSeconds(0.1)
                        //First Chamber set
                         //Todo: Grab open
                        //Reset lift6
                        //Go to push 2 Sample

                        .setTangent(Math.PI)
                        .stopAndAdd(new ArmMotorAction(BigArm,true,0))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_RESET_POSITION))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .splineToLinearHeading(new Pose2d(-45,-55.7,0.8*Math.PI), -Math.PI/2)
                        .stopAndAdd(new ServoAction(forward_slide,0))

                        .waitSeconds(0.1)
                        .stopAndAdd(new ServoAction(claw_heng,0.41))
                        .stopAndAdd(new ServoAction(claw_shu,1))
                        .stopAndAdd(new ServoAction(arm_forward,0.17))
                        .waitSeconds(0.45)
                        .stopAndAdd(new ServoAction(forward_claw,0.4))
                        .waitSeconds(0.27)
                        .stopAndAdd(new ServoAction(claw_shu,0.2))
                        .stopAndAdd(new ServoAction(arm_forward,0.9))
                        .stopAndAdd(new ServoAction(claw_heng,0.55))
                        .waitSeconds(0.6)
                        .stopAndAdd(new ServoAction(forward_claw,1))
                        .waitSeconds(0.15)
                        .stopAndAdd(new ServoAction(forward_slide,0))
                        .splineToLinearHeading(new Pose2d(-48,-58.5,Math.PI),Math.PI/2)
                        .stopAndAdd(new ServoAction(forward_slide,0.9))
                        .waitSeconds(0.1)
                        .stopAndAdd(new ServoAction(claw_shu,1))
                        .stopAndAdd(new ServoAction(arm_forward,0.17))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoAction(forward_claw,0.4))
                        .waitSeconds(0.25)
                        .stopAndAdd(new ServoAction(claw_shu,0.2))
                        .stopAndAdd(new ServoAction(arm_forward,0.9))
                        .waitSeconds(0.6)
                        .stopAndAdd(new ServoAction(forward_claw,1))
                        .waitSeconds(0.15)
                        .stopAndAdd(new ServoAction(forward_slide,0))
                        .splineToConstantHeading(new Vector2d(-48,-47),Math.PI/2)
                        .stopAndAdd(new ServoAction(forward_slide,0.9))
                        .waitSeconds(0.1)
                        .stopAndAdd(new ServoAction(claw_shu,1))
                        .stopAndAdd(new ServoAction(arm_forward,0.17))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoAction(forward_claw,0.4))
                        .waitSeconds(0.25)
                        .stopAndAdd(new ServoAction(claw_shu,0.2))
                        .stopAndAdd(new ServoAction(arm_forward,0.9))
                        .waitSeconds(0.6)
                        .stopAndAdd(new ServoAction(forward_claw,1))
                        .waitSeconds(0.15)
                        .stopAndAdd(new ServoAction(forward_slide,0))
                        .splineToConstantHeading(new Vector2d(-64,-40),Math.PI)
                        .stopAndAdd(new ServoAction(backgrap,0.6))
                        .waitSeconds(0.2)
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .waitSeconds(0.1)
                        .stopAndAdd(new ArmMotorAction(BigArm,false,BIG_ARM_SET_POSITION))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_SET_POSITION))
                        .splineToConstantHeading(new Vector2d(-32, 0), Math.PI/4)
                        .stopAndAdd(new ServoAction(backgrap,0))
                        .waitSeconds(0.1)

                        //Set Sample 2
                        .setTangent(Math.PI)
                        .stopAndAdd(new ArmMotorAction(BigArm,true,0))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_RESET_POSITION))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .splineToConstantHeading(new Vector2d(-64,-40),Math.PI)
                        .stopAndAdd(new ServoAction(backgrap,0.6))
                        .waitSeconds(0.1)
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .waitSeconds(0.05)
                        .stopAndAdd(new ArmMotorAction(BigArm,false,BIG_ARM_SET_POSITION))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_SET_POSITION))
                        .splineToConstantHeading(new Vector2d(-32, 0), 0)
                        .stopAndAdd(new ServoAction(backgrap,0))
                        .waitSeconds(0.1)

                        .setTangent(Math.PI)
                        .stopAndAdd(new ArmMotorAction(BigArm,true,0))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_RESET_POSITION))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .splineToConstantHeading(new Vector2d(-64,-40),Math.PI)
                        .stopAndAdd(new ServoAction(backgrap,0.6))
                        .waitSeconds(0.1)
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .waitSeconds(0.05)
                        .stopAndAdd(new ArmMotorAction(BigArm,false,BIG_ARM_SET_POSITION))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_SET_POSITION))
                        .splineToConstantHeading(new Vector2d(-32, 3), 0)
                        .stopAndAdd(new ServoAction(backgrap,0))
                        .waitSeconds(0.1)

                        .setTangent(Math.PI)
                        .stopAndAdd(new ArmMotorAction(BigArm,true,0))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_RESET_POSITION))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .splineToConstantHeading(new Vector2d(-64,-40),Math.PI)
                        .stopAndAdd(new ServoAction(backgrap,0.6))
                        .waitSeconds(0.1)
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .waitSeconds(0.05)
                        .stopAndAdd(new ArmMotorAction(BigArm,false,BIG_ARM_SET_POSITION))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_SET_POSITION))
                        .splineToConstantHeading(new Vector2d(-32, -3), 0)
                        .stopAndAdd(new ServoAction(backgrap,0))
                        .waitSeconds(0.1)

/*
                        //.setTangent(180)
                        //Todo: Grab open and back arm set down
                        .splineToConstantHeading(new Vector2d(-40,-20),-Math.PI*0.61111)
                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        .splineToConstantHeading(new Vector2d(-61.5,-37),0)
                        //Todo:Grab close
                        //Lift up first
                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))

                        .splineToConstantHeading(new Vector2d(-52,-30),Math.PI*0.0416666)
                        //Todo: Back arm set up
                        .splineToConstantHeading(new Vector2d(-32,0),Math.PI/4)
                        //set sample
                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        //Todo: Grab open and back arm set down
                        .splineToConstantHeading(new Vector2d(-35,-20),-Math.PI*0.61111)
                        .splineToConstantHeading(new Vector2d(-53,-37),Math.PI)
                        .splineToConstantHeading(new Vector2d(-61.5,-37),0)
                        //Todo: Grab close

                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .splineToConstantHeading(new Vector2d(-52,-30),Math.PI*0.0416666)

                        //Todo: Back arm set up
                        .splineToConstantHeading(new Vector2d(-32.5,0),Math.PI/4)
                        //Todo: Grab open and back arm set down
                        //set sample

                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))

                        .splineToConstantHeading(new Vector2d(-35,-20),-Math.PI*0.61111)
                        .splineToConstantHeading(new Vector2d(-53,-37),Math.PI)
                        .splineToConstantHeading(new Vector2d(-61.5,-37),0)
                        //Todo: Grab close
                        //Lift up first
                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_UP_POSITION))
                        .splineToConstantHeading(new Vector2d(-52,-30),Math.PI*0.0416666)
                        //Todo: Back arm set up
                        .splineToConstantHeading(new Vector2d(-32.5,0),Math.PI/4)
                        //Todo: Grab open and back arm set down
                        //set sample
                        */
                        //.stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,LIFT_RESET_POSITION))
                        .stopAndAdd(new ArmMotorAction(BigArm,true,0))
                        .stopAndAdd(new ServoAction(FuckingArm,BACK_ARM_RESET_POSITION))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .splineToConstantHeading(new Vector2d(-60,-60),-Math.PI/2,new TranslationalVelConstraint(200))
                        .build());
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
    public class SlideServoAction implements Action{
        Servo S1 = null;
        Servo S2 = null;
        public SlideServoAction(Servo s1,Servo s2){
            this.S1 = s1;
            this.S2 = s2;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(S1.getPosition() <= 0.01&& S2.getPosition()>=0.54){
                S1.setPosition(0.55);
                S2.setPosition(0.0);
                return false;
            }
            S1.setPosition(0.0);
            S2.setPosition(0.55);
            return false;
        }
    }
    public static class ArmMotorAction implements Action{
        DcMotorEx arm = null;
        boolean IsSet = false;
        int position = 0;
        public ArmMotorAction(DcMotorEx M,boolean F, int p){
            this.arm = M;
            this.IsSet = F;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(IsSet){
                arm.setPower(0.5);
                return false;
            }
            arm.setPower(0.7);
            return false;
        }
    }
    public static class MotorAction implements Action {
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
    public static class ServoAction implements Action {
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
