package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;


@Autonomous(name = "RedWarehouse6CloseRight")
public class RedWarehouse6CloseRight extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence preloadTraj;

    TrajectorySequence firstCycleWareEntry;
    TrajectorySequence firstCycleWareCollect;
    TrajectorySequence firstCycleScore;

    TrajectorySequence secondCycleWareEntry;
    TrajectorySequence secondCycleWareCollect;
    TrajectorySequence secondCycleScore;

    TrajectorySequence thirdCycleWareEntry;
    TrajectorySequence thirdCycleWareCollect;
    TrajectorySequence thirdCycleScore;

    TrajectorySequence fourthCycleWareEntry;
    TrajectorySequence fourthCycleWareCollect;
    TrajectorySequence fourthCycleScore;

    TrajectorySequence fifthCycleWareEntry;
    TrajectorySequence fifthCycleWareCollect;
    TrajectorySequence fifthCycleScore;

    TrajectorySequence sixthCycleWareEntry;
    TrajectorySequence sixthCycleWareCollect;
    TrajectorySequence sixthCycleScore;



    TrajectorySequence entryTraj;

    Pose2d startPose;
    Pose2d newPose;
    Pose2d warehouseEntryPose;

    private DcMotor intakeMotor;
    private DcMotor duckMotor;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseServo;

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorFront;

    private BNO055IMU imu;
    Orientation angles;

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

    public double rightOffset = 0;
    public double getHeadingOffset = 0;

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

    double headingOffset;

    private double initPos = 0;
    private double intakePos = 0.27;

    private Timing.Timer scoreTimer;

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

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
        tseServo = hardwareMap.get(Servo.class, "tseServo");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");


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
            intakeMotor.setPower(-0.6);
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
            intakeMotor.setPower(-0.6);
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
            intakeMotor.setPower(-0.6);
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
            intakeMotor.setPower(1);
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
            intakeMotor.setPower(1);
            depositServo.setPosition(depositIntermediate);
        });

        intakeDown = new Thread(() -> {
            intakeDown();
        });

        startPose = new Pose2d(10, -58, Math.toRadians(270));

        waitForStart();

        while(opModeIsActive()) {

            tseServo.setPosition(0.34);
                drive.setPoseEstimate(startPose);
            preloadTraj = drive.trajectorySequenceBuilder(startPose)
                    .back(5)
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            firstCycleWareEntry = drive.trajectorySequenceBuilder(preloadTraj.end())
                    .splineToLinearHeading(new Pose2d(14, -55, Math.toRadians(0)), Math.toRadians(90))
                    .build();

            firstCycleWareCollect = drive.trajectorySequenceBuilder(firstCycleWareEntry.end())
                    .lineToLinearHeading(new Pose2d(45, -55 , Math.toRadians(360)))
                    .build();

            firstCycleScore = drive.trajectorySequenceBuilder(firstCycleWareCollect.end())
                    .lineToLinearHeading(new Pose2d(14, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(13, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            secondCycleWareEntry = drive.trajectorySequenceBuilder(firstCycleScore.end())
                    .splineToLinearHeading(new Pose2d(14, -55, Math.toRadians(10)), Math.toRadians(360))
                    .build();

            secondCycleWareCollect = drive.trajectorySequenceBuilder(secondCycleWareEntry.end())
                    .lineToLinearHeading(new Pose2d(45, -55 , Math.toRadians(360)))
                    .build();

            secondCycleScore = drive.trajectorySequenceBuilder(secondCycleWareCollect.end())
                    .lineToLinearHeading(new Pose2d(13, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            thirdCycleWareEntry = drive.trajectorySequenceBuilder(secondCycleScore.end())
                    .splineToLinearHeading(new Pose2d(14, -55, Math.toRadians(0)), Math.toRadians(360))
                    .build();

            thirdCycleWareCollect = drive.trajectorySequenceBuilder(thirdCycleWareEntry.end())
                    .lineToLinearHeading(new Pose2d(45, -55 , Math.toRadians(360)))
                    .build();

            thirdCycleScore = drive.trajectorySequenceBuilder(thirdCycleWareCollect.end())
                    .lineToLinearHeading(new Pose2d(13, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            fourthCycleWareEntry = drive.trajectorySequenceBuilder(thirdCycleScore.end())
                    .splineToLinearHeading(new Pose2d(14, -55, Math.toRadians(0)), Math.toRadians(360))
                    .build();

            fourthCycleWareCollect = drive.trajectorySequenceBuilder(fourthCycleWareEntry.end())
                    .lineToLinearHeading(new Pose2d(45, -55 , Math.toRadians(360)))
                    .build();

            fourthCycleScore = drive.trajectorySequenceBuilder(fourthCycleWareCollect.end())
                    .lineToLinearHeading(new Pose2d(13, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            fifthCycleWareEntry = drive.trajectorySequenceBuilder(fourthCycleScore.end())
                    .splineToLinearHeading(new Pose2d(14, -55, Math.toRadians(0)), Math.toRadians(360))
                    .build();

            fifthCycleWareCollect = drive.trajectorySequenceBuilder(fifthCycleWareEntry.end())
                    .lineToLinearHeading(new Pose2d(45, -55 , Math.toRadians(360)))
                    .build();

            fifthCycleScore = drive.trajectorySequenceBuilder(fifthCycleWareCollect.end())
                    .lineToLinearHeading(new Pose2d(13, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            sixthCycleWareEntry = drive.trajectorySequenceBuilder(fifthCycleScore.end())
                    .splineToLinearHeading(new Pose2d(14, -55, Math.toRadians(0)), Math.toRadians(360))
                    .build();

            sixthCycleWareCollect = drive.trajectorySequenceBuilder(sixthCycleWareEntry.end())
                    .lineToLinearHeading(new Pose2d(45, -55, Math.toRadians(360)))
                    .build();

            sixthCycleScore = drive.trajectorySequenceBuilder(sixthCycleWareCollect.end())
                    .lineToLinearHeading(new Pose2d(13, -55, Math.toRadians(365)))
                    .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                    .build();

            drive.followTrajectorySequence(preloadTraj);

            drive.followTrajectorySequence(firstCycleWareEntry);
            drive.followTrajectorySequence(firstCycleWareCollect);
            drive.followTrajectorySequence(firstCycleScore);

            drive.followTrajectorySequence(secondCycleWareEntry);
            drive.followTrajectorySequence(secondCycleWareCollect);
            drive.followTrajectorySequence(secondCycleScore);

            drive.followTrajectorySequence(thirdCycleWareEntry);
            drive.followTrajectorySequence(thirdCycleWareCollect);
            drive.followTrajectorySequence(thirdCycleScore);

            drive.followTrajectorySequence(fourthCycleWareEntry);
            drive.followTrajectorySequence(fourthCycleWareCollect);
            drive.followTrajectorySequence(fourthCycleScore);

            drive.followTrajectorySequence(fifthCycleWareEntry);
            drive.followTrajectorySequence(fifthCycleWareCollect);
            drive.followTrajectorySequence(fifthCycleScore);

            drive.followTrajectorySequence(sixthCycleWareEntry);
            drive.followTrajectorySequence(sixthCycleWareCollect);
            drive.followTrajectorySequence(sixthCycleScore);


            sleep(30000);
        }
    }

    private void CaseC() {
        drive.setPoseEstimate(startPose);
    }

    private void CaseB() {
        drive.setPoseEstimate(startPose);

    }

    private void CaseA() {
        drive.setPoseEstimate(startPose);


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

    public void rampDeposit() {
        depositServo.setPosition(depositRamp);
    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double rightOffset() {
      return rightOffset = distanceSensorRight.getDistance(DistanceUnit.INCH);
    }
    public double headingOffset(){
        return headingOffset = 270 + angles.firstAngle;
    }

}