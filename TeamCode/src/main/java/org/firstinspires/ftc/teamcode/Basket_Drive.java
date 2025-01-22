package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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

@Autonomous(name = "BasketDrive_BY27570",group = "old")
public class Basket_Drive extends LinearOpMode {
    private DcMotorEx Left_Hanging_Motor, Right_Hanging_Motor;
    private Servo backgrap,frame,forward_slide, arm_forward, claw_shu, forward_claw, claw_heng;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive Drive = new MecanumDrive(hardwareMap, new Pose2d(-65,30,Math.PI/2));

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
        backgrap.setPosition(0);
        forward_slide.setPosition(1);
        arm_forward.setPosition(0.8);
        claw_shu.setPosition(0.5);
        forward_claw.setPosition(1);
        telemetry.addData("LLM",Left_Hanging_Motor);
        telemetry.addData("RLM",Right_Hanging_Motor);
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-65,30,Math.PI/2))

                        .setTangent(0)
                        //Lift up first
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,2600))
                        .waitSeconds(0.2)

                        .splineToSplineHeading(new Pose2d(-64,61,Math.PI*(3.0/4.0)),0)
                        .stopAndAdd(new PatientServoAction(frame,0))
                        .waitSeconds(1.5)
                        .stopAndAdd(new PatientServoAction(frame,0.7))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(-Math.PI/4)

                        .splineToSplineHeading(new Pose2d(-40,44,Math.PI),0)

                        .stopAndAdd(new PatientServoAction(claw_shu,1))
                        .stopAndAdd(new PatientServoAction(arm_forward,0.12))
                        .waitSeconds(0.2)
                        .stopAndAdd(new PatientServoAction(forward_claw,0.4))
                        .waitSeconds(0.4)
                        .stopAndAdd(new PatientServoAction(claw_shu,0.2))
                        .stopAndAdd(new PatientServoAction(arm_forward,0.9))
                        .waitSeconds(0.5)
                        .stopAndAdd(new PatientServoAction(forward_claw,1))
                        .waitSeconds(0.2)
                        .stopAndAdd(new ServoAction(claw_shu,0.5))
                        .stopAndAdd(new ServoAction(arm_forward,0.8))
                        .setTangent(Math.PI)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,2600))
                        .waitSeconds(0.2)
                        .splineToSplineHeading(new Pose2d(-64,61,Math.PI*(3.0/4.0)),0)
                        .stopAndAdd(new PatientServoAction(frame,0))
                        .waitSeconds(1.7)
                        .stopAndAdd(new PatientServoAction(frame,0.7))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .setTangent(-Math.PI/4)
                        .splineToSplineHeading(new Pose2d(-40,56,Math.PI),0)

                        .stopAndAdd(new PatientServoAction(claw_shu,1))
                        .stopAndAdd(new PatientServoAction(arm_forward,0.12))
                        .waitSeconds(0.2)
                        .stopAndAdd(new PatientServoAction(forward_claw,0.4))
                        .waitSeconds(0.5)
                        .stopAndAdd(new PatientServoAction(claw_shu,0.2))
                        .stopAndAdd(new PatientServoAction(arm_forward,0.9))
                        .waitSeconds(0.2)
                        .stopAndAdd(new PatientServoAction(forward_claw,1))
                        .waitSeconds(0.2)
                        .stopAndAdd(new ServoAction(claw_shu,0.5))
                        .stopAndAdd(new ServoAction(arm_forward,0.8))
                        .setTangent(Math.PI)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,2600))
                        .waitSeconds(0.2)
                        .splineToSplineHeading(new Pose2d(-64,61,Math.PI*(3.0/4.0)),0)
                        .stopAndAdd(new PatientServoAction(frame,0))
                        .waitSeconds(1.7)
                        .stopAndAdd(new PatientServoAction(frame,0.7))
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))
                        .stopAndAdd(new ServoAction(backgrap,0.5))
                        .setTangent(Math.PI/4)
                        .splineToSplineHeading(new Pose2d(-20, 58, Math.PI), 0)//Ending
                        .splineToSplineHeading(new Pose2d(-6, 17, -Math.PI/2), 0)//Ending
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
            Left_Hanging_Motor.setPower(0.8);
            Right_Hanging_Motor.setPower(0.8);
            return false;
        }
    }
}
