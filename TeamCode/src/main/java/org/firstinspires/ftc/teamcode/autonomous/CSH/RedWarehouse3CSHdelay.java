package org.firstinspires.ftc.teamcode.autonomous.CSH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveSlow;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

@Disabled
@Autonomous(name = "RedWarehouse3CSHdelay")
public class RedWarehouse3CSHdelay extends LinearOpMode {
    SampleMecanumDriveSlow drive;
    TrajectorySequence traj0;
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    TrajectorySequence traj4;
    TrajectorySequence traj5;

    Pose2d startPose;

    private DcMotor intakeMotor;
    private DcMotor duckMotor;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseServo;

    private Thread slideTopThread;
    private Thread slideMidThread;
    private Thread slideLowThread;
    private Thread slideIntermediateThread;
    private Thread scoreThread;
    private Thread intakeDown;
    private Thread scoreThread2;

    public double depositOpen = 0;
    public double depositClose = 0.36;
    public double depositRamp = 0.20;
    public double depositIntermediate = 0.12;

    private double fourBarTopPos = 0.72;
    private double fourBarMidPos = 0.68;
    private double fourBarLowPos = 0.88;
    private double fourBarIntakePos = 0.022;
    private double fourBarIntermediatePos = 0.15;

    private int slideLevel3Pos = 1550;
    private int slideLevel2Pos = 750;
    private int slideLevel1Pos = 831;
    private int slideIntermediate = 1200;
    private int slideIntakePos = 0;

    private double initPos = 0;
    private double intakePos = 0.27;

    private Timing.Timer scoreTimer;

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

    public void runOpMode() {
        drive = new SampleMecanumDriveSlow(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        leftSlideMotor = new Motor(hardwareMap, "leftSlideMotor");
        rightSlideMotor = new Motor(hardwareMap, "rightSlideMotor");

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        tseServo = hardwareMap.get(Servo.class, "tseServo");

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
        tseServo.setPosition(0);

        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, 1);
        webcam.init();


        slideTopThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.8);
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

        slideMidThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.8);
            moveFourBarIntermediate();
            moveSlideMid();
            moveFourBarMiddle();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideLowThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.8);
            moveFourBarIntermediate();
            moveSlideLow();
            moveFourBarLow();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        scoreThread2 = new Thread(() -> {
            depositServo.setPosition(depositRamp);

            scoreTimer = new Timing.Timer(300);
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
            intakeMotor.setPower(0.9);
            depositServo.setPosition(depositIntermediate);
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
            intakeMotor.setPower(0.9);
            depositServo.setPosition(depositIntermediate);
        });

        intakeDown = new Thread(() -> {
            intakeDown();
        });

        startPose = new Pose2d(11, -58, Math.toRadians(270));

        waitForStart();
        while(opModeIsActive()) {
            tseServo.setPosition(0.34);
            barcodePosition = webcam.getBarcodePosition();
            webcam.stopCamera();
            if(barcodePosition == BarCodeDetection.BarcodePosition.LEFT)
                CaseA();
            else if(barcodePosition == BarCodeDetection.BarcodePosition.MIDDLE)
                CaseB();
            else CaseC();
            sleep(30000);
        }

    }

    private void CaseC() {
        drive.setPoseEstimate(startPose);

        traj0 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(5)
                .strafeRight(10)
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(-14, -46, Math.toRadians(270)))
                .build();

        traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .waitSeconds(1.5)
                .addTemporalMarker(1, () -> {
                    scoreThread.start();
                })
                .lineToLinearHeading(new Pose2d(12, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(46, -57, Math.toRadians(360)))
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(14, -58, Math.toRadians(360)))
                .build();

        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(306)))
                .addTemporalMarker(7, () -> {
                    scoreThread.start();
                })
                .build();

        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(14, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(360)))
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(14, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(0, -41, Math.toRadians(312)))
                .addTemporalMarker(7, () -> {
                    scoreThread.start();
                })
                .build();

        traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(13, -56, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(55, -57, Math.toRadians(360)))
                .addTemporalMarker(4, () -> {
                    intakeMotor.setPower(0);
                })
                .build();


        drive.followTrajectorySequence(traj0);
        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);
        drive.followTrajectorySequence(traj4);
    }

    private void CaseB() {
        drive.setPoseEstimate(startPose);

        traj0 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(5)
                .strafeRight(5)
                .addDisplacementMarker(() -> {
                    slideMidThread.start();
                })
                .build();

        traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(0, -39.3, Math.toRadians(310)))
                .addTemporalMarker(0.9, () -> {
                    scoreThread.start();
                })
                .lineToLinearHeading(new Pose2d(12, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(46, -57, Math.toRadians(360)))
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(14, -58, Math.toRadians(360)))
                .build();

        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(306)))
                .addTemporalMarker(7, () -> {
                    scoreThread.start();
                })
                .build();

        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(14, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(360)))
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(14, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(0, -41, Math.toRadians(312)))
                .addTemporalMarker(7, () -> {
                    scoreThread.start();
                })
                .build();

        traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(13, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(55, -57, Math.toRadians(360)))
                .addTemporalMarker(4, () -> {
                    intakeMotor.setPower(0);
                })
                .build();


        drive.followTrajectorySequence(traj0);
        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);
        drive.followTrajectorySequence(traj4);
    }

    private void CaseA() {
        drive.setPoseEstimate(startPose);

        traj0 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(5)
                .strafeRight(10)
                .addDisplacementMarker(() -> {
                    slideLowThread.start();
                })
                .build();

        traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(-10, -43.8, Math.toRadians(290)))
                .addTemporalMarker(0.7, () -> {
                    scoreThread2.start();
                })
                .lineToLinearHeading(new Pose2d(12, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(46, -57, Math.toRadians(360)))
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(14, -56, Math.toRadians(360)))
                .build();

        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(306)))
                .addTemporalMarker(7, () -> {
                    scoreThread.start();
                })
                .build();

        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(14, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(360)))
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .lineToLinearHeading(new Pose2d(14, -56, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(0, -41, Math.toRadians(312)))
                .addTemporalMarker(7, () -> {
                    scoreThread.start();
                })
                .build();

        traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(13, -57, Math.toRadians(360)))
                .lineToLinearHeading(new Pose2d(55, -58, Math.toRadians(360)))
                .addTemporalMarker(4, () -> {
                    intakeMotor.setPower(0);
                })
                .build();


        drive.followTrajectorySequence(traj0);
        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);
        drive.followTrajectorySequence(traj4);
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

    public void moveSlideMid() {
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel2Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel2Pos);

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

    public void moveSlideLow() {
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel1Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel1Pos);

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

    public void rampDeposit(){
        depositServo.setPosition(depositRamp);
    }

}