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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Autonomous(name = "First_Auto_ChamberDrive_BY27570_TestDrive",group = "old")
public class Test_Drive extends LinearOpMode {
    private Servo backgrap;
    private DcMotorEx Left_Hanging_Motor, Right_Hanging_Motor;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive Drive = new MecanumDrive(hardwareMap, new Pose2d(-62.5,-9.5,0));
        //Servo FrontSlide = hardwareMap.servo.get("TTT");
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
        backgrap.setPosition(0);
        waitForStart();

        Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(-62.5,-9.5,0))
                        .setTangent(0)
                        //Lift up first
                        .setTangent(0)
                        //Lift up first
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,1450))
                        .splineToConstantHeading(new Vector2d(-33.75, -9.5), 0)
                        //First Chamber set
                        .stopAndAdd(new PatientMotorAction(Left_Hanging_Motor,Right_Hanging_Motor,930))
                        .stopAndAdd(new PatientServoAction(backgrap,0.7))
                        .waitSeconds(0.3)
                        .stopAndAdd(new MotorAction(Left_Hanging_Motor,Right_Hanging_Motor,0))

                        .setTangent(180)
                        .splineToConstantHeading(new Vector2d(-59,-55),-Math.PI/2)
                        .build());
    }public class PatientServoAction implements Action {
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
            Left_Hanging_Motor.setPower(0.8);
            Right_Hanging_Motor.setPower(0.8);
            return false;
        }
    }
}
