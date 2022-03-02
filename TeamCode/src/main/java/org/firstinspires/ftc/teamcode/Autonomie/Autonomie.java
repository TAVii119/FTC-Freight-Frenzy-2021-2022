package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousTest")
public class Autonomie extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence traj0;
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    TrajectorySequence trajtest;
    Pose2d startPose;
    Pose2d secondPose;

    private DcMotor intakeMotor;
    private DcMotor duckMotor;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;

    private Thread slideTopThread;
    private Thread slideMidThread;
    private Thread slideLowThread;
    private Thread slideIntermediateThread;
    private Thread scoreThread;
    private Thread intakeDown;

    public double depositOpen = 0;
    public double depositClose = 0.36;
    public double depositRamp = 0.09;
    public double depositIntermediate = 0.12;

    private double fourBarTopPos = 0.72;
    private double fourBarMidPos = 0.68;
    private double fourBarLowPos = 0.86;
    private double fourBarIntakePos = 0.022;
    private double fourBarIntermediatePos = 0.15;

    private int slideLevel3Pos = 1550;
    private int slideLevel2Pos = 800;
    private int slideLevel1Pos = 831;
    private int slideIntermediate = 1200;
    private int slideIntakePos = 0;

    private double initPos = 0;
    private double intakePos = 0.27;

    Timing.Timer scoreTimer;

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        leftSlideMotor = new Motor(hardwareMap, "leftSlideMotor");
        rightSlideMotor = new Motor(hardwareMap, "rightSlideMotor");

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");

        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlideMotor.resetEncoder();
        rightSlideMotor.resetEncoder();

        // Invert motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        duckMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlideMotor.setInverted(true);

        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);

        iLifterServo.setPosition(0);
        depositServo.setPosition(0.12);
        gbServoLeft.setPosition(0.022);
        gbServoRight.setPosition(0.022);

        slideTopThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.5);
            moveFourBarIntermediate();
            moveSlideTop();
            moveFourBarTop();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        scoreThread = new Thread(() -> {
            depositServo.setPosition(depositOpen);

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            moveFourBarIntermediate();

            scoreTimer = new Timing.Timer(800);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            moveSlideIntake();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            moveFourBarIntake();
            intakeMotor.setPower(0.5);
            depositServo.setPosition(depositIntermediate);
        });
        intakeDown = new Thread(() -> {
            intakeDown();
        });

        startPose = new Pose2d(10, -58, Math.toRadians(270));

        waitForStart();
        while(opModeIsActive()) {
            drive.setPoseEstimate(startPose);

            traj0 = drive.trajectorySequenceBuilder(startPose)
                    .back(10)
                    .build();

            traj1 = drive.trajectorySequenceBuilder(traj0.end())
                    .lineToLinearHeading(new Pose2d(4, -42, Math.toRadians(300)))
                    .lineToLinearHeading(new Pose2d(10, -61, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(45, -64, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(370)))
                    .build();

            trajtest = drive.trajectorySequenceBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(4, -41, Math.toRadians(320)))
                    .build();

            traj2 = drive.trajectorySequenceBuilder(trajtest.end())
                    .lineToLinearHeading(new Pose2d(12, -62, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -64, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -64, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(4, -41, Math.toRadians(320)))
                    .build();

            traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .lineToLinearHeading(new Pose2d(12, -64, Math.toRadians(335)))
                    .lineToLinearHeading(new Pose2d(48, -66, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -64, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(4, -41, Math.toRadians(280)))
                    .lineToLinearHeading(new Pose2d(12, -65, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -67, Math.toRadians(340)))
                    .build();

            drive.followTrajectorySequence(traj0);
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(trajtest);
            drive.followTrajectorySequence(traj2);
            drive.followTrajectorySequence(traj3);
            sleep(30000);
        }

    }
    public void moveSlideTop() {
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel3Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel3Pos);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(1);
            rightSlideMotor.set(1);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }

    public void moveSlideIntake() {
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set the target position
        leftSlideMotor.setTargetPosition(slideIntakePos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideIntakePos);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(1);
            rightSlideMotor.set(1);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }

    public void moveFourBarTop() {
        gbServoRight.setPosition(fourBarTopPos);
        gbServoLeft.setPosition(fourBarTopPos);
    }

    public void moveFourBarMiddle() {
        gbServoRight.setPosition(fourBarMidPos);
        gbServoLeft.setPosition(fourBarMidPos);
    }

    public void moveFourBarLow() {
        gbServoRight.setPosition(fourBarLowPos);
        gbServoLeft.setPosition(fourBarLowPos);
    }

    public void moveFourBarIntermediate() {
        gbServoRight.setPosition(fourBarIntermediatePos);
        gbServoLeft.setPosition(fourBarIntermediatePos);
    }

    public void moveFourBarIntake() {
        gbServoRight.setPosition(fourBarIntakePos);
        gbServoLeft.setPosition(fourBarIntakePos);
    }

    public void intakeDown(){
        iLifterServo.setPosition(0);
    }

}
