package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousTest")
public class Autonomie extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence traj1;
    Pose2d startPose;

    private DcMotor intakeMotor;
    private DcMotor duckMotor;
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;

    private Thread slideTopThread;
    private Thread slideMidThread;
    private Thread slideLowThread;
    private Thread slideIntermediateThread;
    private Thread scoreThread;

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
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        startPose = new Pose2d(-35, -58, Math.toRadians(270));

        depositServo.setPosition(0);
        gbServoLeft.setPosition(0);
        gbServoRight.setPosition(0);

        slideTopThread = new Thread(() -> {
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeMotor.setPower(-0.5);
            moveFourBarIntermediate();
            moveSlideTop();
            moveFourBarTop();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideMidThread = new Thread(() -> {
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeMotor.setPower(-0.5);
            moveFourBarIntermediate();
            moveSlideMiddle();
            moveFourBarMiddle();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideLowThread = new Thread(() -> {
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeMotor.setPower(-0.5);
            moveFourBarIntermediate();
            moveSlideLow();
            moveFourBarLow();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideIntermediateThread = new Thread(() -> {
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeMotor.setPower(-0.5);
            moveFourBarIntermediate();
            moveSlideIntermediate();

            scoreTimer = new Timing.Timer(200);
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

            scoreTimer = new Timing.Timer(500);
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

        waitForStart();
        while(opModeIsActive()) {
            drive.setPoseEstimate(startPose);
            traj1 = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(-12, -47))
                    .lineToLinearHeading(new Pose2d(10, -60.5, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(370)))
                    .lineToLinearHeading(new Pose2d(4, -44, Math.toRadians(300)))

                    .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(335)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(380)))
                    .lineToLinearHeading(new Pose2d(4, -44, Math.toRadians(300)))

                    .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(380)))
                    .lineToLinearHeading(new Pose2d(4, -44, Math.toRadians(300)))
                    .lineToLinearHeading(new Pose2d(10, -60.5, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .build();

            drive.followTrajectorySequence(traj1);
            sleep(30000);
        }
    }

    public void moveSlideTop() {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setTargetPosition(slideLevel3Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel3Pos);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
        while(leftSlideMotor.isBusy() && rightSlideMotor.isBusy()) {

        }

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    public void moveSlideMiddle() {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setTargetPosition(slideLevel2Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel2Pos);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
        while(leftSlideMotor.isBusy() && rightSlideMotor.isBusy()) {

        }

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    public void moveSlideLow() {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setTargetPosition(slideLevel1Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel1Pos);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
        while(leftSlideMotor.isBusy() && rightSlideMotor.isBusy()) {

        }

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    public void moveSlideIntermediate() {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setTargetPosition(slideIntermediate); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideIntermediate);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
        while(leftSlideMotor.isBusy() && rightSlideMotor.isBusy()) {

        }

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    public void moveSlideIntake() {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setTargetPosition(slideIntakePos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideIntakePos);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
        while(leftSlideMotor.isBusy() && rightSlideMotor.isBusy()) {

        }

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
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
}

