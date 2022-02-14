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
import org.firstinspires.ftc.teamcode.Vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.Vision.BarcodeUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous(name = "AutonomieTimisoara")
public class AutonomieTimisoara extends LinearOpMode {
    private DcMotor intakeMotor;
    private DcMotor duckMotor;

    private Servo depositServo;
    private Servo deposit2;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseServo;

    BarcodeUtil webcamUtil;
    BarCodeDetection.BarcodePosition TSEPosition;

    @Override
    public void runOpMode()
    {
        webcamUtil = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry);
        webcamUtil.init();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        duckMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        deposit2 = hardwareMap.get(Servo.class, "deposit2Servo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        tseServo = hardwareMap.get(Servo.class, "tseServo");

        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        tseServo.setDirection(Servo.Direction.REVERSE);

        depositServo.setPosition(0.16);
        deposit2.setPosition(0);
        iLifterServo.setPosition(0);
        gbServoLeft.setPosition(0.028);
        gbServoRight.setPosition(0.028);
        tseServo.setPosition(0.02);

        waitForStart();

        while (opModeIsActive())
        {
            TSEPosition = webcamUtil.getBarcodePosition();
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

            if(TSEPosition == BarCodeDetection.BarcodePosition.RIGHT)
                caseC(drive);
            else if(TSEPosition == BarCodeDetection.BarcodePosition.MIDDLE)
                caseB(drive);
            else if(TSEPosition == BarCodeDetection.BarcodePosition.LEFT)
                caseA(drive);
            else
                caseC(drive);
            sleep(30000);
        }
    }

    Pose2d startPose = new Pose2d(-12, -62, Math.toRadians(270));

    // Y e spre perete

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, -44))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-57.5, -59, Math.toRadians(190)))
                .forward(0.5)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .back(25)
                .turn(Math.toRadians(150))
                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(345)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj4.end())
                .forward(35)
                .build();

        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .back(35)
                .lineToLinearHeading(new Pose2d(-10.5, -41.5, Math.toRadians(270)))
                .build();

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(5, -66, Math.toRadians(350)))
                .turn(-Math.toRadians(5))
                .forward(37)
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(180);
        goToLevel3();
        sleep(1200);
        pushDeposit();
        useIntake();

        drive.followTrajectorySequence(traj2);

        runCarousel();
        sleep(800);
        runCarousel();

        drive.followTrajectorySequence(traj4);
        useIntake();
        drive.followTrajectory(traj7);
        goToLevel3();
        drive.followTrajectorySequence(traj8);
        sleep(100);
        pushDeposit();
        useIntake();
        drive.followTrajectorySequence(traj11);
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, -52))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-57.5, -59, Math.toRadians(190)))
                .forward(1)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .back(25)
                .turn(Math.toRadians(150))
                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(345)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj4.end())
                .forward(35)
                .build();

        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .back(35)
                .lineToLinearHeading(new Pose2d(-10.5 , -41, Math.toRadians(270)))
                .build();

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(5, -66, Math.toRadians(350)))
                .turn(-Math.toRadians(5))
                .forward(38)
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(180);
        goToLevel2();
        sleep(1350);
        pushDeposit();
        useIntake();

        drive.followTrajectorySequence(traj2);

        runCarousel();
        sleep(800);
        runCarousel();

        drive.followTrajectorySequence(traj4);
        useIntake();
        drive.followTrajectory(traj7);
        goToLevel3();
        drive.followTrajectorySequence(traj8);
        sleep(100);
        pushDeposit();
        useIntake();
        drive.followTrajectorySequence(traj11);
    }

    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, -52))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-57.5, -59, Math.toRadians(190)))
                .forward(1.3)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .back(25)
                .turn(Math.toRadians(150))
                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(345)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj4.end())
                .forward(36.5)
                .build();

        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .back(35)
                .lineToLinearHeading(new Pose2d(-10.5, -41.5, Math.toRadians(270)))
                .build();

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(5, -66, Math.toRadians(350)))
                .turn(-Math.toRadians(5))
                .forward(38)
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(180);
        goToLevel1();
        sleep(1500);
        pushDeposit();
        useIntake();

        drive.followTrajectorySequence(traj2);

        runCarousel();
        sleep(800);
        runCarousel();

        drive.followTrajectorySequence(traj4);
        useIntake();
        drive.followTrajectory(traj7);
        sleep(300);
        goToLevel3();
        drive.followTrajectorySequence(traj8);
        sleep(100);
        pushDeposit();
        useIntake();
        drive.followTrajectorySequence(traj11);
    }

    public void goToLevel1() {
        deposit2.setPosition(0);
        intakeMotor.setPower(-0.5);
        iLifterServo.setPosition(0.38);
        sleep(500);
        depositServo.setPosition(0.43);
        gbServoRight.setPosition(0.83);
        gbServoLeft.setPosition(0.83);
    }

    public void goToLevel2() {
        deposit2.setPosition(0);
        intakeMotor.setPower(-0.5);
        iLifterServo.setPosition(0.38);
        sleep(500);
        depositServo.setPosition(0.43);
        gbServoRight.setPosition(0.72);
        gbServoLeft.setPosition(0.72);
    }

    public void goToLevel3() {
        deposit2.setPosition(0);
        intakeMotor.setPower(-0.5);
        iLifterServo.setPosition(0.38);
        sleep(500);
        depositServo.setPosition(0.43);
        gbServoRight.setPosition(0.58);
        gbServoLeft.setPosition(0.58);
    }

    public void pushDeposit() {
        deposit2.setPosition(0.28);
        depositServo.setPosition(0.64);

        // Wait for minerals to be ejected from deposit
        sleep(300);

        deposit2.setPosition(0);
        gbServoLeft.setPosition(0.028);
        gbServoRight.setPosition(0.028);
        intakeMotor.setPower(0.5);
        depositServo.setPosition(0.16);

        sleep(1100);

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