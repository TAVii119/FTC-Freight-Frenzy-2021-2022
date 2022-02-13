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

import java.util.Arrays;

@Autonomous(name = "Autonomous Test")
public class AutonomousTest extends LinearOpMode {
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
        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
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

    Pose2d startPose = new Pose2d(-12, -62, Math.toRadians(265.0));

    // Y e spre perete

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, -42))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-61, -57, Math.toRadians(190)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(2)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(30)
                .lineToSplineHeading(new Pose2d(5, -55, Math.toRadians(355)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(37, -68))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(40)
                .lineToSplineHeading(new Pose2d(-12, -37, Math.toRadians(270)))
                .build();


        Trajectory traj11 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(5, -68, Math.toRadians(360)))
                .lineTo(new Vector2d(39, -68))
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(180);
        goToLevel3();
        sleep(1000);
        pushDeposit();
        useIntake();

        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        runCarousel();
        sleep(800);
        runCarousel();

        drive.followTrajectory(traj4);
        useIntake();
        drive.followTrajectory(traj7);
        useIntake();
        goToLevel3();
        drive.followTrajectory(traj8);
        pushDeposit();
        drive.followTrajectory(traj11);
        useIntake();
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-15, -51))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-61, -57, Math.toRadians(190)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(1)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(30)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -55, Math.toRadians(355)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .strafeRight(10)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(37, -68))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(40)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .strafeLeft(20)
                .build();

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(-8, -33, Math.toRadians(250)))
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(5, -55, Math.toRadians(355)))
                .build();

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(5, -68, Math.toRadians(330)))
                .build();

        Trajectory traj13 = drive.trajectoryBuilder(traj12.end())
                .lineTo(new Vector2d(37, -68))
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(180);
        goToLevel2();
        sleep(1500);
        pushDeposit();
        useIntake();

        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        runCarousel();
        sleep(800);
        runCarousel();

        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        useIntake();
        drive.followTrajectory(traj7);
        useIntake();
        goToLevel3();
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        pushDeposit();
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj12);
        useIntake();
        drive.followTrajectory(traj13);
    }

    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        // Declare trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-15, -51.5))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-61, -57, Math.toRadians(190)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(2)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(30)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -55, Math.toRadians(355)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .strafeRight(10)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(37, -68))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(40)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .strafeLeft(20)
                .build();

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(-6, -32, Math.toRadians(250)))
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(5, -55, Math.toRadians(355)))
                .build();

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(5, -68, Math.toRadians(330)))
                .build();

        Trajectory traj13 = drive.trajectoryBuilder(traj12.end())
                .lineTo(new Vector2d(37, -68))
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        liftIntake();
        sleep(180);
        goToLevel1();
        sleep(1800);
        pushDeposit();
        useIntake();

        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        runCarousel();
        sleep(800);
        runCarousel();

        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        useIntake();
        drive.followTrajectory(traj7);
        useIntake();
        goToLevel3();
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        pushDeposit();
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj12);
        useIntake();
        drive.followTrajectory(traj13);
    }

    public void goToLevel1() {
        deposit2.setPosition(0);
        depositServo.setPosition(0.43);
        intakeMotor.setPower(-0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.83);
        gbServoLeft.setPosition(0.83);
    }

    public void goToLevel2() {
        deposit2.setPosition(0);
        depositServo.setPosition(0.43);
        intakeMotor.setPower(-0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.72);
        gbServoLeft.setPosition(0.72);
    }

    public void goToLevel3() {
        deposit2.setPosition(0);
        depositServo.setPosition(0.43);
        intakeMotor.setPower(-0.5);
        iLifterServo.setPosition(0.38);
        sleep(500);
        gbServoRight.setPosition(0.58);
        gbServoLeft.setPosition(0.58);
    }

    public void pushDeposit() {
        deposit2.setPosition(0.28);
        depositServo.setPosition(0.64);

        // Wait for minerals to be ejected from deposit
        sleep(300);

        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
        intakeMotor.setPower(0.5);
        depositServo.setPosition(0.16);

        sleep(1100);

        iLifterServo.setPosition(0.20);
        deposit2.setPosition(0);
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