package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous(name = "Russia Remote")
public class AutonomousRussia2 extends LinearOpMode {
    private DcMotor intakeMotor;
    private DcMotor duckMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseServo;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        duckMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        tseServo = hardwareMap.get(Servo.class, "tseServo");

        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        tseServo.setDirection(Servo.Direction.REVERSE);

        depositServo.setPosition(0.16);
        iLifterServo.setPosition(0);
        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
        tseServo.setPosition(0.02);

        waitForStart();

        while (opModeIsActive())
        {
            final int robotRadius = 9; // inches
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

//            if (pipeline.position == RingsDeterminationPipeline.RingPosition.NONE) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseA(drive);
//                sleep(30000);
//            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.ONE) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseB(drive);
//                sleep(30000);
//            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.FOUR) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseC(drive);
//                sleep(30000);
//            } else {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseA(drive);
//                sleep(30000);
//            }

            caseC(drive);
            sleep(30000);
        }
    }

    Pose2d startPose = new Pose2d(-63.0, 49.0, Math.toRadians(0.0));

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .back(20)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .forward(5)
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(250);
        goToLevel3();
        sleep(1000);
        pushDeposit();

        drive.followTrajectory(traj2);

    }

    public void goToLevel1() {
        depositServo.setPosition(0.38);
        intakeMotor.setPower(0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.83);
        gbServoLeft.setPosition(0.83);
    }

    public void goToLevel2() {
        depositServo.setPosition(0.38);
        intakeMotor.setPower(0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.72);
        gbServoLeft.setPosition(0.72);
    }

    public void goToLevel3() {
        depositServo.setPosition(0.38);
        intakeMotor.setPower(0.5);
        iLifterServo.setPosition(0.38);
        sleep(500);
        gbServoRight.setPosition(0.57);
        gbServoLeft.setPosition(0.57);
    }

    public void pushDeposit() {
        depositServo.setPosition(0.64);

        // Wait for minerals to be ejected from deposit
        sleep(250);

        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
        intakeMotor.setPower(0.5);
        depositServo.setPosition(0.16);

        sleep(650);

        iLifterServo.setPosition(0.20);
    }

    public void useIntake() {
        if(intakeMotor.getPower() != 0) {
            intakeMotor.setPower(0);
        } else {
            intakeMotor.setPower(0.5);
        }
    }

    public void runCarousel() {
        if (duckMotor.getPower() != 0)
            duckMotor.setPower(0);
        else
        {
            duckMotor.setPower(0.65);
            sleep(800);
            duckMotor.setPower(1);
        }
    }

    public void liftIntake() {
        if (iLifterServo.getPosition() == 0.0)
            iLifterServo.setPosition(0.2);
    }
}