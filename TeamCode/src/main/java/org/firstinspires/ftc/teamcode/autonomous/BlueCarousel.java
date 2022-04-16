package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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



@Autonomous(name = "BlueCarousel", group = "Blue Carousel")
public class BlueCarousel extends LinearOpMode {

    SampleMecanumDrive drive;
    Trajectory traj1;
    Trajectory traj2Top;
    Trajectory traj2Mid;
    Trajectory traj2Low;
    Trajectory traj3Low;
    Trajectory traj3Mid;
    Trajectory traj3Top;
    TrajectorySequence traj4;
    Trajectory traj5;
    TrajectorySequence traj6;
    TrajectorySequence traj7;
    TrajectorySequence traj8Top;
    TrajectorySequence traj8Mid;
    TrajectorySequence traj8Low;
    TrajectorySequence traj9Top;
    TrajectorySequence traj9Mid;
    TrajectorySequence traj9Low;
    TrajectorySequence traj10;
    Trajectory traj11;
    TrajectorySequence traj12;
    TrajectorySequence traj13;
    Trajectory traj15;

    TrajectorySequence entryTraj;

    Pose2d startPose;
    Pose2d newPose;
    Pose2d warehouseEntryPose;

    private DcMotor lb;
    private DcMotor lf;
    private DcMotor rb;
    private DcMotor rf;

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
    private Thread scoreThreadFinal;
    private Thread intakeDown;
    private Thread scoreThread2;
    private Thread CaseC;
    private Thread sensorOuttake;

    public double depositOpen = 0;
    public double depositClose = 0.34;
    public double depositRamp = 0.16;
    public double depositIntermediate = 0.12;

    public double rightOffset = 0;
    public double getHeadingOffset = 0;

    private double fourBarTopPos = 0.76;
    private double fourBarMidPos = 0.75;
    private double fourBarLowPos = 0.78;
    private double fourBarIntakePos = 0.01;
    private double fourBarIntermediatePos = 0.13;
    private double fourBarIntermediateScorePos = 0.03;

    private int slideLevel3Pos = 1550;
    private int slideLevel2Pos = 710;
    private int slideLevel1Pos = 358;
    private int slideIntermediate = 540;
    private int slideIntakePos = 0;

    double headingOffset;

    private double initPos = 0;
    private double intakePos = 0.27;

    private Timing.Timer scoreTimer;

    private BarcodeUtil detector;
    private BarCodeDetection.BarcodePosition barcodePosition = BarCodeDetection.BarcodePosition.NOT_FOUND;

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

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
        rightSlideMotor.setInverted(true);

        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        tseArmServo.setDirection(Servo.Direction.REVERSE);

        iLifterServo.setPosition(0);
        depositServo.setPosition(0.30);
        gbServoLeft.setPosition(0.022);
        gbServoRight.setPosition(0.022);
        tseArmServo.setPosition(0.02);
        tseClawServo.setPosition(0);

        BarcodeUtil detector = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, 1);
        detector.init();


        slideTopThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
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

        slideMidThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
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

        slideIntermediateThread = new Thread(() -> {
            iLifterServo.setPosition(0.27);
            depositServo.setPosition(depositClose);

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.35);
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

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            intakeMotor.setPower(-0.1);
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

            scoreTimer = new Timing.Timer(1000);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(450);
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

        scoreThread = new Thread(() -> {
            depositServo.setPosition(depositOpen);

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(450);
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

        scoreThreadFinal = new Thread(() -> {
            depositServo.setPosition(depositOpen);

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            moveFourBarIntermediateScore();

            scoreTimer = new Timing.Timer(450);
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

            intakeMotor.setPower(0);
            depositServo.setPosition(depositIntermediate);
        });

        sensorOuttake = new Thread(() -> {
            depositServo.setPosition(depositClose);
            sleep(100);
            intakeMotor.setPower(-0.3);
        });

        intakeDown = new Thread(() -> {
            intakeDown();
        });

        startPose = new Pose2d(-35, 58, -Math.toRadians(270));

        traj1 = drive.trajectoryBuilder(startPose)
                .back(5)
                .build();

      /*  traj2Top = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(140)))
                .build();

        traj2Mid = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-24, -38.2, Math.toRadians(240)))
                .build();

        traj2Low = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-24, -38.5, Math.toRadians(240)))
                .build();

        traj3Top = drive.trajectoryBuilder(traj2Top.end())
                .lineToLinearHeading(new Pose2d(-45, -50, Math.toRadians(190)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();*/

        traj4 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-62, 51, Math.toRadians(60)))
                .forward(1,   SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        traj5 = drive.trajectoryBuilder(traj4.end())
                .back(4)
                .build();


        traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .turn(Math.toRadians(30))
                .build();

        traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(-55, 19, -Math.toRadians(270)))
                .build();

        traj8Top = drive.trajectorySequenceBuilder(traj7.end())
                .turn(Math.toRadians(80))
                .lineToLinearHeading(new Pose2d( -39 , 21, -Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj8Mid = drive.trajectorySequenceBuilder(traj7.end())
                .turn(Math.toRadians(80))
                .lineToLinearHeading(new Pose2d( -33.9 , 21, -Math.toRadians(190)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj8Low = drive.trajectorySequenceBuilder(traj7.end())
                .turn(Math.toRadians(80))
                .lineToLinearHeading(new Pose2d( -33.9 , 21, -Math.toRadians(190)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj9Top = drive.trajectorySequenceBuilder(traj8Top.end())
                .forward(17)
                .turn(-Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-64, 33, -Math.toRadians(360)))
                .back(5)
                .build();

        traj9Mid = drive.trajectorySequenceBuilder(traj8Mid.end())
                .forward(15)
                .turn(-Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-64, 33, -Math.toRadians(360)))
                .back(5)
                .build();


        traj9Low = drive.trajectorySequenceBuilder(traj8Low.end())
                .forward(15)
                .turn(-Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-64, 31, -Math.toRadians(360)))
                .back(5)
                .build();

        traj10 = drive.trajectorySequenceBuilder(traj9Low.end())
                .lineToLinearHeading(new Pose2d(-64, 37, -Math.toRadians(360)))
                .back(5)
                .build();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Element position", detector.getBarcodePosition());
            telemetry.update();
            barcodePosition = detector.getBarcodePosition();
        }

        Thread stopCamera = new Thread(() -> detector.stopCamera());
        stopCamera.start();

        waitForStart();
        tseArmServo.setPosition(0.20);

        if(!isStopRequested()) {
            if(barcodePosition == BarCodeDetection.BarcodePosition.LEFT)
                CaseA(drive);
            else if(barcodePosition == BarCodeDetection.BarcodePosition.MIDDLE)
                CaseB(drive);
            else
                CaseC(drive);
            sleep(30000);
        }
    }

    private void CaseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        drive.followTrajectory(traj1);
        drive.followTrajectorySequence(traj4);
        sleep(300);
        duckMotor.resetEncoder();
        duckMotor.setRunMode(Motor.RunMode.RawPower);

        while (duckMotor.getCurrentPosition() < 1200)
            duckMotor.set(0.25);

        sleep(50);
        duckMotor.stopMotor();
        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), -Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        slideTopThread.start();
        drive.followTrajectorySequence(traj8Top);
        scoreThread.start();
        sleep(500);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), -Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));
        drive.followTrajectorySequence(traj9Top);
        tseArmServo.setPosition(0);
        iLifterServo.setPosition(0);
    }

    private void CaseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        drive.followTrajectory(traj1);
        drive.followTrajectorySequence(traj4);
        sleep(300);
        duckMotor.resetEncoder();
        duckMotor.setRunMode(Motor.RunMode.RawPower);

        while (duckMotor.getCurrentPosition() < 1200)
            duckMotor.set(0.25);

        sleep(50);
        duckMotor.stopMotor();
        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), -Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));

        slideMidThread.start();
        drive.followTrajectorySequence(traj8Mid);
        scoreThread.start();
        sleep(500);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), -Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));
        drive.followTrajectorySequence(traj9Mid);
        tseArmServo.setPosition(0);
        iLifterServo.setPosition(0);
    }

    private void CaseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        drive.followTrajectory(traj1);
        drive.followTrajectorySequence(traj4);
        sleep(300);
        duckMotor.resetEncoder();
        duckMotor.setRunMode(Motor.RunMode.RawPower);

        while (duckMotor.getCurrentPosition() < 1200)
            duckMotor.set(0.25);

        sleep(50);
        duckMotor.stopMotor();
        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), -Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));
        slideLowThread.start();

        drive.followTrajectorySequence(traj8Low);
        scoreThread2.start();

        sleep(500);

        newPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(newPose.getX(), newPose.getY(), -Math.toRadians(270) + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle));
        drive.followTrajectorySequence(traj9Low);

        tseArmServo.setPosition(0);
        iLifterServo.setPosition(0);
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

    public void moveFourBarIntermediateScore() {
        gbServoLeft.setPosition(fourBarIntermediateScorePos);
        gbServoRight.setPosition(fourBarIntermediateScorePos);
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