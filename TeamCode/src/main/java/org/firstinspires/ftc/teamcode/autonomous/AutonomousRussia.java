package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Autonomous", group="")
public class AutonomousRussia extends LinearOpMode  {
    private SampleMecanumDrive drive;

    private Motor intakeMotor;
    private Motor duckMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo tseServo;

    Timing.Timer scoreTimer;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        duckMotor = new Motor(hardwareMap, "duckMotor");

        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        duckMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setInverted(true);
        duckMotor.setInverted(true);

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

        while(opModeIsActive()) {

        }
    }

    public void goToLevel1() {
        depositServo.setPosition(0.38);
        intakeMotor.set(0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.83);
        gbServoLeft.setPosition(0.83);
    }

    public void goToLevel2() {
        depositServo.setPosition(0.38);
        intakeMotor.set(0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.72);
        gbServoLeft.setPosition(0.72);
    }

    public void goToLevel3() {
        depositServo.setPosition(0.38);
        intakeMotor.set(0.5);
        iLifterServo.setPosition(0.38);

        gbServoRight.setPosition(0.57);
        gbServoLeft.setPosition(0.57);
    }

    public void PushDeposit() {
        depositServo.setPosition(0.64);

        // Wait for minerals to be ejected from deposit
        scoreTimer = new Timing.Timer(250);
        scoreTimer.start();

        while (!scoreTimer.done())
        {
            // Wait for timer to end
        }
        scoreTimer.pause();

        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
        intakeMotor.set(0.5);
        depositServo.setPosition(0.16);

        scoreTimer = new Timing.Timer(650);
        scoreTimer.start();
        while (!scoreTimer.done())
        {
            // Wait for timer to end
        }
        scoreTimer.pause();

        iLifterServo.setPosition(0.20);
    }

    public void useIntake() {
        if(intakeMotor.get() != 0) {
            intakeMotor.set(0);
        } else {
            intakeMotor.set(0.5);
        }
    }

    public void runCarousel() {
        if (duckMotor.get() != 0)
            duckMotor.set(0);
        else
        {
            scoreTimer = new Timing.Timer(800);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                duckMotor.set(0.65);
            }
            scoreTimer.pause();
            duckMotor.set(1);
        }
    }

    public void liftIntake() {
        if (iLifterServo.getPosition() == 0.0)
            iLifterServo.setPosition(0.2);
    }
}
