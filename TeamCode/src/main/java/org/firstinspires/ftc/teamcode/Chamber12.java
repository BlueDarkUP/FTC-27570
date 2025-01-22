package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual_Path_Trigger")
public class Chamber12 extends LinearOpMode {

    private DcMotorEx Left_Hanging_Motor, Right_Hanging_Motor;
    private static final  int LIFT_UP_POSITION = 1300;
    private static final  int LIFT_DOWN_POSITION = 500;
    private static final int LIFT_RESET_POSITION = 0;

    private static final int WCXS= 6;
    private Servo backgrap,frame,forward_slide, arm_forward, claw_shu, forward_claw, claw_heng;

    private MecanumDrive Drive;
    private boolean isExecutingPath = false; // Flag to prevent re-triggering

    @Override
    public void runOpMode() throws InterruptedException {
        Drive = new MecanumDrive(hardwareMap, new Pose2d(-66.0, 64.0, -Math.PI / 2));
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

        while (opModeIsActive()) {
            if (gamepad1.dpad_left && !isExecutingPath) {
                isExecutingPath = true;
                runMyPath();
                isExecutingPath = false; // Reset the flag after execution
            }
            telemetry.update(); // Keep telemetry updating
        }
    }

    private void runMyPath() {
        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-66.0, 64.0, -Math.PI / 2))
                        .setTangent(0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 62.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-10.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-10.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.5), 0.0)
                        .splineToSplineHeading(new Pose2d(-8.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-8.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-6.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-6.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.5), 0.0)
                        .splineToSplineHeading(new Pose2d(-4.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-4.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-2.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-2.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(0.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(0.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(2.0,  35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(2.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(4.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(4.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(6.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(6.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(8.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(8.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(10.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(10.0, 32.0), 0.0)
                        .splineToSplineHeading(new Pose2d(-50.0, 62.0, -Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(-50.0, 64.0), 0.0)
                        .splineToSplineHeading(new Pose2d(12.0, 35.0, Math.PI / 2), 0.0)
                        .splineToConstantHeading(new Vector2d(12.0, 32.0), 0.0)
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