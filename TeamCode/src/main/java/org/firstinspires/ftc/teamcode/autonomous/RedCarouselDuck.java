package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.Timing;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;


@Autonomous(name = "RedCarouselDuck", group = "Red Carousel")
public class RedCarouselDuck extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence preloadTop;
    TrajectorySequence preloadMid;
    TrajectorySequence preloadLow;

    TrajectorySequence traj1Top;
    TrajectorySequence traj1Mid;
    TrajectorySequence traj1Low;
    TrajectorySequence carouselLow;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    TrajectorySequence traj4;
    TrajectorySequence traj5;
    TrajectorySequence trajNoDuck;


    Pose2d startPose;
    Pose2d newPose;

    private DcMotor intakeMotor;
    private Motor duckMotor;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseArmServo;
    private Servo tseClawServo;

    private BNO055IMU imu;
    private Orientation angles;

    private RevColorSensorV3 colorSensor;

    private Thread slideTopDuckThread;
    private Thread slideTopThread;
    private Thread slideMidThread;
    private Thread slideLowThread;
    private Thread slideIntermediateThread;
    private Thread scoreThread;
    private Thread intakeDown;
    private Thread scoreThreadRamp;
    private Thread scoreThreadNoIntake;
    private Thread scoreDuckThread;


    public double depositOpen = 0;
    public double depositClose = 0.30;
    public double depositCloseDuck = 0.42;
    public double depositRamp = 0.18;
    public double depositIntermediate = 0.12;

    private double fourBarTopPos = 0.76;
    private double fourBarMidPos = 0.75;
    private double fourBarLowPos = 0.78;
    private double fourBarIntakePos = 0.005;
    private double fourBarIntermediatePos = 0.13;
    private double fourBarIntermediateScorePos = 0.02;
    private double fourBarWait = 0.19;

    private int slideLevel3Pos = 1550;
    private int slideLevel2Pos = 710;
    private int slideLevel1Pos = 350;
    private int slideIntermediate = 1200;
    private int slideIntakePos = 0;

    private double initPos = 0;
    private double intakePos = 0.27;

    private boolean duckFound = false;

    private Timing.Timer scoreTimer;

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        duckMotor = new Motor(hardwareMap, "duckMotor");
        leftSlideMotor = new Motor(hardwareMap, "leftSlideMotor");
        rightSlideMotor = new Motor(hardwareMap, "rightSlideMotor");

        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        tseArmServo = hardwareMap.get(Servo.class, "tseArmServo");
        tseClawServo = hardwareMap.get(Servo.class, "tseClawServo");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "cupSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlideMotor.resetEncoder();
        rightSlideMotor.resetEncoder();
        duckMotor.resetEncoder();
        duckMotor.setInverted(true);

        // Invert motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlideMotor.setInverted(true);

        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        tseArmServo.setDirection(Servo.Direction.REVERSE);

        iLifterServo.setPosition(0);
        depositServo.setPosition(0.30);
        gbServoLeft.setPosition(0.022);
        gbServoRight.setPosition(0.022);
        tseArmServo.setPosition(0);
        tseClawServo.setPosition(0);

        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, 1); // 1 - Delta TSE, 2 - Soft TSE, 3 - T TSE
        webcam.init();

        slideTopDuckThread = new Thread(() -> {
            iLifterServo.setPosition(0.285);
            depositServo.setPosition(depositCloseDuck);

            scoreTimer = new Timing.Timer(600);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            movefourBarWait();
            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            moveSlideTopDuck();
            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            intakeMotor.setPower(0);
        });

        slideTopThread = new Thread(() -> {
            iLifterServo.setPosition(0.285);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.35);
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

        slideMidThread = new Thread(() -> {
            iLifterServo.setPosition(0.285);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.35);
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

        scoreThreadRamp = new Thread(() -> {
            depositServo.setPosition(depositRamp);

            scoreTimer = new Timing.Timer(800);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(800);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            moveSlideIntake();
            moveFourBarIntake();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            depositServo.setPosition(depositIntermediate);
        });

        scoreThread = new Thread(() -> {
            depositServo.setPosition(depositOpen);

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(150);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveSlideIntake();
            moveFourBarIntake();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            depositServo.setPosition(depositIntermediate);
        });

        scoreThreadNoIntake = new Thread(() -> {
            depositServo.setPosition(depositOpen);

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveSlideIntake();
            moveFourBarIntake();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            depositServo.setPosition(depositIntermediate);
        });

        scoreDuckThread = new Thread(() ->{
            moveFourBarTop();
            scoreTimer = new Timing.Timer(800);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            depositServo.setPosition(depositOpen);

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(450);
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
            depositServo.setPosition(depositIntermediate);
        });

        intakeDown = new Thread(() -> {
            intakeDown();
        });
        Thread carouselThread = new Thread(() -> {
            duckMotor.resetEncoder();
            duckMotor.setRunMode(Motor.RunMode.RawPower);
            while (duckMotor.getCurrentPosition() < 1200)
                duckMotor.set(0.25);
            sleep(50);
            duckMotor.stopMotor();

        });

        Thread carouselThreadLow = new Thread(() -> {
            duckMotor.resetEncoder();
            duckMotor.setRunMode(Motor.RunMode.RawPower);
            while (duckMotor.getCurrentPosition() < 1200)
                duckMotor.set(0.2);
            sleep(50);
            duckMotor.stopMotor();

        });


        startPose = new Pose2d(-35, -58, -Math.toRadians(90));
        preloadTop = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    slideTopThread.start();
                })
                .back(5)
                .lineToLinearHeading(new Pose2d(-30, -36.5, -Math.toRadians(140)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        preloadMid = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    slideMidThread.start();
                })
                .back(5)
                .lineToLinearHeading(new Pose2d(-23.5, -37.5, -Math.toRadians(130)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        preloadLow = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    slideLowThread.start();
                })
                .back(5)
                .lineToLinearHeading(new Pose2d(-24, -36, -Math.toRadians(130)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        carouselLow = drive.trajectorySequenceBuilder(preloadLow.end())
                .lineToLinearHeading(new Pose2d(-45, -50, -Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(-64, -55, Math.toRadians(150)))
                .waitSeconds(3)
                .addTemporalMarker( 2.3, () -> carouselThread.start())
                .build();

        traj1Top = drive.trajectorySequenceBuilder(preloadTop.end())
                .lineToLinearHeading(new Pose2d(-45, -50, -Math.toRadians(120)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-56, -57, Math.toRadians(150)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(4)
                .addTemporalMarker( 3.3, () -> carouselThread.start())
                .build();

        traj1Mid = drive.trajectorySequenceBuilder(preloadMid.end())
                .lineToLinearHeading(new Pose2d(-45, -50, -Math.toRadians(120)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-56, -57, Math.toRadians(150)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(3)
                .addTemporalMarker( 3.2, () -> carouselThread.start())
                .build();

        traj1Low = drive.trajectorySequenceBuilder(preloadLow.end())
                .lineToLinearHeading(new Pose2d(-45, -50, -Math.toRadians(120)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-56, -57, Math.toRadians(150)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(3.5)
                .addTemporalMarker( 3.5, () -> carouselThread.start())
                .build();

        traj2 = drive.trajectorySequenceBuilder(traj1Top.end())
                .lineToLinearHeading(new Pose2d(-25, -56.5, -Math.toRadians(120)))
                .addDisplacementMarker(() -> intakeMotor.setPower(1))
                .lineToConstantHeading(new Vector2d(-58, -59),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .turn(Math.toRadians(40))
                .lineToConstantHeading(new Vector2d(-40, -58),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .turn(Math.toRadians(-40))
                .lineToConstantHeading(new Vector2d(-61, -58),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .addDisplacementMarker(() -> {
                    slideTopDuckThread.start();
                })
                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-57, -52, -Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-58, -19, -Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(-39, -20),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(-38, -16))
                .turn(-Math.toRadians(180))
                .back(4)
                .lineToConstantHeading(new Vector2d(-42, -37))
                .addTemporalMarker(6,() -> tseArmServo.setPosition(0))
                .addTemporalMarker(6,() -> iLifterServo.setPosition(0))
                .back(30)
                .build();

        trajNoDuck = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-61, -35.2, Math.toRadians(0)))
                .back(10)
                .build();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Element position", webcam.getBarcodePosition());
            telemetry.update();
            barcodePosition = webcam.getBarcodePosition();
        }
        Thread stopCamera = new Thread(() -> webcam.stopCamera());
        stopCamera.start();

        waitForStart();

        tseArmServo.setPosition(0.20);
        sleep(1000);
        if(barcodePosition == BarCodeDetection.BarcodePosition.LEFT)
            CaseA();
        else if(barcodePosition == BarCodeDetection.BarcodePosition.MIDDLE)
            CaseB();
        else
            CaseC();
        sleep(30000);
    }

    private void CaseC() {
        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(preloadTop);

        scoreThreadNoIntake.start();
        sleep(500);
        drive.followTrajectorySequence(traj1Top);

        drive.followTrajectorySequenceAsync(traj2);


        while(drive.isBusy() && opModeIsActive()) {
            if(depositgetDistance() <  5) {
                duckFound = true;
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                sleep(150);
                depositServo.setPosition(depositCloseDuck);
                moveFourBarIntermediate();
                intakeMotor.setPower(0);
            }
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();

            drive.update();
        }

        sleep(500);
        if(duckFound || depositgetDistance() < 5) {
            drive.followTrajectorySequence(traj3);

            sleep(500);
            drive.update();
            newPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

            drive.followTrajectorySequence(traj4);

            scoreDuckThread.start();
            drive.followTrajectorySequence(traj5);
        }
        else {
            sleep(500);
            drive.update();
            newPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

            drive.followTrajectorySequence(trajNoDuck);
            tseArmServo.setPosition(0.01);
            iLifterServo.setPosition(0);
            intakeMotor.setPower(0);
        }
    }

    private void CaseB() {
        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(preloadMid);

        scoreThreadNoIntake.start();
        sleep(500);
        drive.followTrajectorySequence(traj1Mid);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));


        drive.followTrajectorySequenceAsync(traj2);


        while(drive.isBusy() && opModeIsActive()) {
            if(depositgetDistance() <  5) {
                duckFound = true;
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                sleep(150);
                depositServo.setPosition(depositCloseDuck);
                moveFourBarIntermediate();
                intakeMotor.setPower(0);
            }
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();

            drive.update();
        }

        if(duckFound || depositgetDistance() < 5) {
            drive.followTrajectorySequence(traj3);

            sleep(500);
            drive.update();
            newPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

            drive.followTrajectorySequence(traj4);

            scoreDuckThread.start();
            drive.followTrajectorySequence(traj5);
        }
        else {
            sleep(500);
            drive.update();
            newPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

            drive.followTrajectorySequence(trajNoDuck);
            tseArmServo.setPosition(0.01);
            iLifterServo.setPosition(0);
            intakeMotor.setPower(0);
        }
    }

    private void CaseA() {
        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(preloadLow);

        scoreThreadNoIntake.start();
        sleep(500);
        drive.followTrajectorySequence(traj1Low);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        drive.followTrajectorySequenceAsync(traj2);

        while(drive.isBusy() && opModeIsActive()) {
            if(depositgetDistance() <  5) {
                duckFound = true;
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                sleep(150);
                depositServo.setPosition(depositCloseDuck);
                moveFourBarIntermediate();
                intakeMotor.setPower(0);
            }
            telemetry.addData("Distance", depositgetDistance());
            telemetry.update();

            drive.update();
        }

        if(duckFound || depositgetDistance() < 5) {
            drive.followTrajectorySequence(traj3);

            sleep(500);
            drive.update();
            newPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

            drive.followTrajectorySequence(traj4);

            scoreDuckThread.start();
            drive.followTrajectorySequence(traj5);
        }
        else {
            sleep(500);
            drive.update();
            newPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

            drive.followTrajectorySequence(trajNoDuck);
            tseArmServo.setPosition(0.01);
            iLifterServo.setPosition(0);
            intakeMotor.setPower(0);
        }
    }

    public void moveSlideTopDuck() {
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
        while (!leftSlideMotor.atTargetPosition() && !isStopRequested()) {
            leftSlideMotor.set(0.5);
            rightSlideMotor.set(0.5);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
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
        while (!leftSlideMotor.atTargetPosition() && !isStopRequested()) {
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
        while (!leftSlideMotor.atTargetPosition() && !isStopRequested()) {
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
        while (!leftSlideMotor.atTargetPosition() && !isStopRequested()) {
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
        while (!leftSlideMotor.atTargetPosition() && !isStopRequested()) {
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

    public void movefourBarWait(){
        gbServoLeft.setPosition(fourBarWait);
        gbServoRight.setPosition(fourBarWait);
    }
    public void rampDeposit(){
        depositServo.setPosition(depositRamp);
    }
    public void moveFourBarIntermediateScore() {
        gbServoLeft.setPosition(fourBarIntermediateScorePos);
        gbServoRight.setPosition(fourBarIntermediateScorePos);
    }

    private double depositgetDistance() {
        return colorSensor.getDistance(DistanceUnit.CM);
    }
}