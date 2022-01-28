package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DepositCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="TeleOperated")

public class TeleOperated extends CommandOpMode {
    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;

    private Motor intakeMotor;
    private Motor slideMotor;
    private Motor turretMotor;
    private Motor duckMotor;

    private Servo depositServo;
    private Servo armServo;
    private Servo iLifterServo;

    // Declare commands and subsystems
    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    private CarouselSubsystem carouselSubsystem;

    private DepositSubsystem depositSubsystem;
    private DepositCommand depositCommand;

    private TurretSubsystem turretSubsystem;
    private TurretCommand turretCommand;

    private SlideSubsystem slideSubsystem;
    private SlideCommand slideCommand;

    private ArmSubsystem armSubsystem;

    private InstantCommand startCarouselCommand;
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;
    private InstantCommand useSlideCommand;
    private InstantCommand scoreCommand;
    private InstantCommand returnSlideCommand;
    private InstantCommand moveTurretLeft;
    private InstantCommand moveTurretRight;
    private InstantCommand extendSlide;
    private InstantCommand retractSlide;

    GamepadEx driver1;

    @Override
    public void initialize() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station).
        lf = new Motor(hardwareMap, "lf");
        rf = new Motor(hardwareMap, "rf");
        lb = new Motor(hardwareMap, "lb");
        rb = new Motor(hardwareMap, "rb");

        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        turretMotor = new Motor(hardwareMap, "turretMotor");
        slideMotor = new Motor(hardwareMap, "slideMotor");
        duckMotor = new Motor(hardwareMap, "duckMotor");

        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        duckMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        armServo = hardwareMap.get(Servo.class, "armServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        depositServo = hardwareMap.get(Servo.class, "depositServo");

        // Initialize motors and Servos
        depositSubsystem.initServo();
        armSubsystem.initServo();
        iLifterServo.setPosition(0);

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);

        // Initialize subsystems and commands
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem);

        carouselSubsystem = new CarouselSubsystem(duckMotor);

        turretSubsystem = new TurretSubsystem(turretMotor);
        turretCommand = new TurretCommand(turretSubsystem);

        slideSubsystem = new SlideSubsystem(slideMotor);
        slideCommand = new SlideCommand(slideSubsystem);

        armSubsystem = new ArmSubsystem(armServo);

        // Instant commands

        intakeRunCommand = new InstantCommand(() -> {
            if(intakeSubsystem.getIntakePower() == 0.0) {
                intakeSubsystem.runIntake();
            }
            else {
                intakeSubsystem.stopIntake();
            }
        }, intakeSubsystem);

        intakeReverseCommand = new InstantCommand(() -> {
            intakeSubsystem.reverseIntake();
        }, intakeSubsystem);

        useSlideCommand = new InstantCommand(() -> {
            intakeSubsystem.reverseIntake();
            depositSubsystem.closeDeposit();
            armSubsystem.extendArm();
            slideSubsystem.moveToHub();
        });

        returnSlideCommand = new InstantCommand(() -> {
            slideSubsystem.moveToHome();
            armSubsystem.retractArm();
            intakeSubsystem.runIntake();
        });

        scoreCommand = new InstantCommand(() -> {
            depositSubsystem.openDeposit();
        });

        moveTurretLeft = new InstantCommand(() -> {
            turretMotor.set(0.1);
        });

        moveTurretRight = new InstantCommand(() -> {
            turretMotor.set(-0.1);
        });

        extendSlide = new InstantCommand(() -> {
            slideMotor.set(0.1);
        });

        retractSlide = new InstantCommand(() -> {
            slideMotor.set(-0.1);
        });


        Button moveTurretLeftButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(moveTurretLeft);
        Button moveTurretRightButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(moveTurretRight);
        Button extendSlideButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(extendSlide);
        Button retractSlideButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(retractSlide);

        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);

        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(startCarouselCommand);

        Button useSlideButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(useSlideCommand);
        Button scoreButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(scoreCommand);
        Button returnSlideButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(returnSlideCommand);

        // Register subsystems and set their default commands
        register(driveSubsystem, depositSubsystem, intakeSubsystem, carouselSubsystem, slideSubsystem, turretSubsystem, armSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        slideSubsystem.setDefaultCommand(slideCommand);
        turretSubsystem.setDefaultCommand(turretCommand);
        depositSubsystem.setDefaultCommand(depositCommand);
    }
}