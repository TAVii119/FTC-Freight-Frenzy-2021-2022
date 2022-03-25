package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;



@Autonomous(name = "BlueWarehouse5")
public class BlueWarehouse5 extends LinearOpMode {
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

    private DcMotor lb;
    private DcMotor lf;
    private DcMotor rb;
    private DcMotor rf;

    private DcMotor intakeMotor;
    private DcMotor duckMotor;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseArmServo;
    private Servo tseClawServo;

    private RevColorSensorV3 colorSensor;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorFront;

    private BNO055IMU imu;
    private Orientation angles;

    private Thread slideTopThread;
    private Thread slideMidThread;
    private Thread slideLowThread;
    private Thread slideIntermediateThread;
    private Thread scoreThread;
    private Thread intakeDown;
    private Thread scoreThread2;
    private Thread CaseC;


    public double depositOpen = 0;
    public double depositClose = 0.36;
    public double depositRamp = 0.20;
    public double depositIntermediate = 0.12;

    public double rightOffset = 0;
    public double getHeadingOffset = 0;

    private double fourBarTopPos = 0.72;
    private double fourBarMidPos = 0.70;
    private double fourBarLowPos = 0.92;
    private double fourBarIntakePos = 0.022;
    private double fourBarIntermediatePos = 0.15;

    private int slideLevel3Pos = 1550;
    private int slideLevel2Pos = 750;
    private int slideLevel1Pos = 870;
    private int slideIntermediate = 540;
    private int slideIntakePos = 0;

    double headingOffset;

    private double initPos = 0;
    private double intakePos = 0.27;

    private Timing.Timer scoreTimer;

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition = BarCodeDetection.BarcodePosition.NOT_FOUND;

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        leftSlideMotor = new Motor(hardwareMap, "leftSlideMotor");
        rightSlideMotor = new Motor(hardwareMap, "rightSlideMotor");

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        tseArmServo = hardwareMap.get(Servo.class, "tseArmServo");
        tseClawServo = hardwareMap.get(Servo.class, "tseClawServo");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "cupSensor");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

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
        tseArmServo.setDirection(Servo.Direction.REVERSE);

        iLifterServo.setPosition(0);
        depositServo.setPosition(0.12);
        gbServoLeft.setPosition(0.022);
        gbServoRight.setPosition(0.022);
        tseArmServo.setPosition(0);
        tseClawServo.setPosition(0);

        BarcodeUtil detector = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, 1);
        detector.init();


        slideTopThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.6);
            moveFourBarIntermediate();
            moveSlideTop();
            moveFourBarTop();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideMidThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.6);
            moveFourBarIntermediate();
            moveSlideMid();
            moveFourBarMiddle();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideIntermediateThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.6);
            moveFourBarIntermediate();
            moveSlideIntermediate();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
        });

        slideLowThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.6);
            moveFourBarIntermediate();
            moveSlideLow();
            moveFourBarLow();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        scoreThread2 = new Thread(() -> {
            depositServo.setPosition(depositRamp);

            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediate();

            scoreTimer = new Timing.Timer(800);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveSlideIntake();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

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
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediate();

            scoreTimer = new Timing.Timer(250);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveSlideIntake();

            scoreTimer = new Timing.Timer(50);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            moveFourBarIntake();
            intakeMotor.setPower(1);
            depositServo.setPosition(depositIntermediate);
        });

        intakeDown = new Thread(() -> {
            intakeDown();
        });

        startPose = new Pose2d(12, 58, Math.toRadians(90));

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Element position", detector.getBarcodePosition());
            telemetry.update();
            barcodePosition = detector.getBarcodePosition();
            detector.stopCamera();
        }

        waitForStart();
        tseArmServo.setPosition(0.20);
        CaseA(drive);
    }

    private void CaseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);
        preloadTraj = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .addTemporalMarker(0, () -> {
                    slideTopThread.start();
                })
                .build();

        firstCycleWareEntry = drive.trajectorySequenceBuilder(preloadTraj.end())
                .lineToLinearHeading(new Pose2d(-5, 42, -Math.toRadians(290)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(0.2)
                .addTemporalMarker(1.3, () -> {
                    scoreThread.start();
                })
                .splineToSplineHeading(new Pose2d(20, 58, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(45, 60), Math.toRadians(360))
                .build();

        firstCycleWareCollect = drive.trajectorySequenceBuilder(firstCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(50, 60, -Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        firstCycleScore = drive.trajectorySequenceBuilder(firstCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 60, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        secondCycleWareEntry = drive.trajectorySequenceBuilder(firstCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 58.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(47, 62), Math.toRadians(360))
                .build();

        secondCycleWareCollect = drive.trajectorySequenceBuilder(secondCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(51, 62, -Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        secondCycleScore = drive.trajectorySequenceBuilder(secondCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        thirdCycleWareEntry = drive.trajectorySequenceBuilder(secondCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 58, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(48, 63), Math.toRadians(360))
                .build();

        thirdCycleWareCollect = drive.trajectorySequenceBuilder(thirdCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 59, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        thirdCycleScore = drive.trajectorySequenceBuilder(thirdCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        fourthCycleWareEntry = drive.trajectorySequenceBuilder(thirdCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 59.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(49, 62, -Math.toRadians(380)), -Math.toRadians(360))
                .build();

        fourthCycleWareCollect = drive.trajectorySequenceBuilder(fourthCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 58, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        fourthCycleScore = drive.trajectorySequenceBuilder(fourthCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(2.7, () -> scoreThread.start())

                .setReversed(false)
                .build();

        fifthCycleWareEntry = drive.trajectorySequenceBuilder(fourthCycleScore.end())
                .splineToSplineHeading(new Pose2d(20, 59.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(49, 62, -Math.toRadians(380)), -Math.toRadians(360))
                .build();

        fifthCycleWareCollect = drive.trajectorySequenceBuilder(fifthCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 57, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        fifthCycleScore = drive.trajectorySequenceBuilder(fifthCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(30, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(2.9, () -> scoreThread.start())
                .setReversed(false)
                .build();

        sixthCycleWareEntry = drive.trajectorySequenceBuilder(fifthCycleScore.end())
                .splineToSplineHeading(new Pose2d(20, 58.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(50, 62), Math.toRadians(360))
                .build();

        drive.followTrajectorySequence(preloadTraj);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(firstCycleWareEntry);
        drive.followTrajectorySequenceAsync(firstCycleWareCollect);
        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() < 4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(firstCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(secondCycleWareEntry);
        drive.followTrajectorySequenceAsync(secondCycleWareCollect);
        while(drive.isBusy()) {
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();

            drive.update();
        }

        drive.followTrajectorySequence(secondCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(thirdCycleWareEntry);
        drive.followTrajectorySequenceAsync(thirdCycleWareCollect);

        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(thirdCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(fourthCycleWareEntry);
        drive.followTrajectorySequenceAsync(fourthCycleWareCollect);
        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(fourthCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(fifthCycleWareEntry);
    }

    private void CaseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);
        preloadTraj = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .addTemporalMarker(0, () -> {
                    slideMidThread.start();
                })
                .build();

        firstCycleWareEntry = drive.trajectorySequenceBuilder(preloadTraj.end())
                .lineToLinearHeading(new Pose2d(-5, 41.5, -Math.toRadians(290)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(0.3)
                .addTemporalMarker(1, () -> {
                    scoreThread.start();
                })
                .splineToSplineHeading(new Pose2d(20, 58, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(45, 60), Math.toRadians(360))
                .build();

        firstCycleWareCollect = drive.trajectorySequenceBuilder(firstCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(50, 60, -Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        firstCycleScore = drive.trajectorySequenceBuilder(firstCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 60, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        secondCycleWareEntry = drive.trajectorySequenceBuilder(firstCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 58.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(47, 62), Math.toRadians(360))
                .build();

        secondCycleWareCollect = drive.trajectorySequenceBuilder(secondCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(51, 62, -Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        secondCycleScore = drive.trajectorySequenceBuilder(secondCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 61.5, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        thirdCycleWareEntry = drive.trajectorySequenceBuilder(secondCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 58, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(48, 63), Math.toRadians(360))
                .build();

        thirdCycleWareCollect = drive.trajectorySequenceBuilder(thirdCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 59, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        thirdCycleScore = drive.trajectorySequenceBuilder(thirdCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        fourthCycleWareEntry = drive.trajectorySequenceBuilder(thirdCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 59.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(49, 62, -Math.toRadians(380)), -Math.toRadians(360))
                .build();

        fourthCycleWareCollect = drive.trajectorySequenceBuilder(fourthCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 58, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        fourthCycleScore = drive.trajectorySequenceBuilder(fourthCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(2.7, () -> scoreThread.start())

                .setReversed(false)
                .build();

        fifthCycleWareEntry = drive.trajectorySequenceBuilder(fourthCycleScore.end())
                .splineToSplineHeading(new Pose2d(20, 59.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(49, 62, -Math.toRadians(380)), -Math.toRadians(360))
                .build();

        fifthCycleWareCollect = drive.trajectorySequenceBuilder(fifthCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 57, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        fifthCycleScore = drive.trajectorySequenceBuilder(fifthCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(30, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(2.9, () -> scoreThread.start())
                .setReversed(false)
                .build();

        sixthCycleWareEntry = drive.trajectorySequenceBuilder(fifthCycleScore.end())
                .splineToSplineHeading(new Pose2d(20, 58.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(50, 62), Math.toRadians(360))
                .build();

        drive.followTrajectorySequence(preloadTraj);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(firstCycleWareEntry);
        drive.followTrajectorySequenceAsync(firstCycleWareCollect);
        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() < 4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(firstCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(secondCycleWareEntry);
        drive.followTrajectorySequenceAsync(secondCycleWareCollect);
        while(drive.isBusy()) {
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();

            drive.update();
        }

        drive.followTrajectorySequence(secondCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(thirdCycleWareEntry);
        drive.followTrajectorySequenceAsync(thirdCycleWareCollect);

        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(thirdCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(fourthCycleWareEntry);
        drive.followTrajectorySequenceAsync(fourthCycleWareCollect);
        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(fourthCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(fifthCycleWareEntry);
    }

    public void CaseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);
        preloadTraj = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .addTemporalMarker(0, () -> {
                    slideLowThread.start();
                })
                .build();

        firstCycleWareEntry = drive.trajectorySequenceBuilder(preloadTraj.end())
                .lineToLinearHeading(new Pose2d(-5, 41.2, -Math.toRadians(295)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(0.3)
                .addTemporalMarker(1.2, () -> {
                    scoreThread2.start();
                })
                .splineToSplineHeading(new Pose2d(20, 59, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(45, 62), Math.toRadians(360))
                .build();

        firstCycleWareCollect = drive.trajectorySequenceBuilder(firstCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(50, 60, -Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        firstCycleScore = drive.trajectorySequenceBuilder(firstCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 60, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        secondCycleWareEntry = drive.trajectorySequenceBuilder(firstCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 58.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(47, 62), Math.toRadians(360))
                .build();

        secondCycleWareCollect = drive.trajectorySequenceBuilder(secondCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(51, 62, -Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        secondCycleScore = drive.trajectorySequenceBuilder(secondCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 61.5, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        thirdCycleWareEntry = drive.trajectorySequenceBuilder(secondCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 58, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(48, 63), Math.toRadians(360))
                .build();

        thirdCycleWareCollect = drive.trajectorySequenceBuilder(thirdCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 59, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        thirdCycleScore = drive.trajectorySequenceBuilder(thirdCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(4.7, () -> scoreThread.start())
                .build();

        fourthCycleWareEntry = drive.trajectorySequenceBuilder(thirdCycleScore.end())
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20, 59.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(49, 62, -Math.toRadians(380)), -Math.toRadians(360))
                .build();

        fourthCycleWareCollect = drive.trajectorySequenceBuilder(fourthCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 58, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        fourthCycleScore = drive.trajectorySequenceBuilder(fourthCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(35, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(2.7, () -> scoreThread.start())

                .setReversed(false)
                .build();

        fifthCycleWareEntry = drive.trajectorySequenceBuilder(fourthCycleScore.end())
                .splineToSplineHeading(new Pose2d(20, 59.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(49, 62, -Math.toRadians(380)), -Math.toRadians(360))
                .build();

        fifthCycleWareCollect = drive.trajectorySequenceBuilder(fifthCycleWareEntry.end())
                .lineToLinearHeading(new Pose2d(55, 57, -Math.toRadians(380)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        fifthCycleScore = drive.trajectorySequenceBuilder(fifthCycleWareCollect.end())
                .lineToLinearHeading(new Pose2d(30, 63, -Math.toRadians(360)))
                .setReversed(true)
                .splineTo(new Vector2d(5, 39.5), -Math.toRadians(127),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> slideTopThread.start())
                .addTemporalMarker(2.9, () -> scoreThread.start())
                .setReversed(false)
                .build();

        sixthCycleWareEntry = drive.trajectorySequenceBuilder(fifthCycleScore.end())
                .splineToSplineHeading(new Pose2d(20, 58.5, -Math.toRadians(360)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(50, 62), Math.toRadians(360))
                .build();

        drive.followTrajectorySequence(preloadTraj);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(firstCycleWareEntry);
        drive.followTrajectorySequenceAsync(firstCycleWareCollect);
        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() < 4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(firstCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(secondCycleWareEntry);
        drive.followTrajectorySequenceAsync(secondCycleWareCollect);
        while(drive.isBusy()) {
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();

            drive.update();
        }

        drive.followTrajectorySequence(secondCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(thirdCycleWareEntry);
        drive.followTrajectorySequenceAsync(thirdCycleWareCollect);

        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(thirdCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(fourthCycleWareEntry);
        drive.followTrajectorySequenceAsync(fourthCycleWareCollect);
        while(drive.isBusy()) {
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();
            if(depositgetDistance() <  4) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                depositServo.setPosition(depositClose);
                sleep(100);
                intakeMotor.setPower(-0.6);
            }

            drive.update();
        }

        drive.followTrajectorySequence(fourthCycleScore);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(90) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequence(fifthCycleWareEntry);
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
            leftSlideMotor.set(0.8);
            rightSlideMotor.set(0.8);
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

    public void moveSlideIntermediate() {
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set the target position
        leftSlideMotor.setTargetPosition(slideIntermediate); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideIntermediate);

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
        return distanceSensorRight.getDistance(DistanceUnit.INCH);
    }
    public double headingOffset(){
        return headingOffset = 270 + angles.firstAngle;
    }

    private double depositgetDistance() {
        return colorSensor.getDistance(DistanceUnit.CM);
    }
    private void driveTime(int milliseconds) {
        Timing.Timer scoreTimer = new Timing.Timer(milliseconds);
        scoreTimer.start();
        while (!scoreTimer.done())
        {
            lb.setPower(-0.5);
            lf.setPower(0.5);
            rf.setPower(-0.5);
            rb.setPower(0.5);
        }
        scoreTimer.pause();
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }


}