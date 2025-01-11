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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Autonomous(name = "Push_Two_Add_Two_Auto_ChamberDrive_BY27570_OPTr",group = "old")
public class Chamber_Drive extends LinearOpMode {

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
        frame.setPosition(0);
        backgrap.setPosition(0);
        forward_slide.setPosition(0.88);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(0.5);
        telemetry.addData("LLM",Left_Hanging_Motor);
        telemetry.addData("RLM",Right_Hanging_Motor);
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-62.5,-9.5,0))
                        .setTangent(0)
                        //Lift up first
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .splineToConstantHeading(new Vector2d(-33.75, -9.5), 0)
                        //First Chamber set
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .stopAndAdd(new PatientServoAction(backgrap,0.7))
                        .waitSeconds(0.3)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        //Reset lift
                        //Go to push 3 Sample
                        .setTangent(180)
                        .splineToSplineHeading(new Pose2d(-46, -22, Math.PI/2), -Math.PI/2)
                        .splineToSplineHeading(new Pose2d(-38, -34, Math.PI), 0)
                        .splineToConstantHeading(new Vector2d(-25, -37), 0)
                        .splineToConstantHeading(new Vector2d(-13, -43), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-20, -48), Math.PI)
                        .splineToConstantHeading(new Vector2d(-48, -48), Math.PI)
                        //1
                        .splineToConstantHeading(new Vector2d(-20, -41), 0)
                        .splineToConstantHeading(new Vector2d(-13, -50.5), -Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-20, -59), Math.PI)
                        .splineToConstantHeading(new Vector2d(-45, -59), Math.PI)
                        //ahead is about 2 sample



                        .splineToConstantHeading(new Vector2d(-50, -37), Math.PI)
                        //Wait for human player to set the sample
                        .waitSeconds(0.1)
                        .splineToConstantHeading(new Vector2d(-61.5, -37), Math.PI)
                        //Get Sample
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        .waitSeconds(0.21)
                        //Lift up first
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(-33.5, -6, 0), 0)
                        .waitSeconds(0.1)
                        //Set Sample 2
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.2)
                        .stopAndAdd(new PatientServoAction(backgrap,0.7))
                        .waitSeconds(0.3)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(180)

                        .splineToSplineHeading(new Pose2d(-53,-37,Math.PI),Math.PI)
                        .waitSeconds(0.1)
                        .splineToConstantHeading(new Vector2d(-61,-37),0)
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        .waitSeconds(0.21)

                        //Lift up first
                        .setTangent(0)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .splineToSplineHeading(new Pose2d(-33.4,-3,0),0)
                        .waitSeconds(0.1)
                        //set sample
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.2)
                        .stopAndAdd(new PatientServoAction(backgrap,0.7))
                        .waitSeconds(0.3)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(180)


                        .splineToSplineHeading(new Pose2d(-53,-37,Math.PI),Math.PI)
                        .waitSeconds(0.1)
                        .splineToConstantHeading(new Vector2d(-61,-37),0)
                        //.stopAndAdd(new PatientServoAction(backgrap,GRAB_CLOSE))
                        .stopAndAdd(new PatientServoAction(backgrap,0))
                        //3
                        .setTangent(0)
                        .waitSeconds(0.3)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .splineToSplineHeading(new Pose2d(-33.3,-0,0),0)
                        //set sample
                        .waitSeconds(0.1)
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .waitSeconds(0.2)
                        .stopAndAdd(new PatientServoAction(backgrap,0.7))
                        .waitSeconds(0.3)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(180)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
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
