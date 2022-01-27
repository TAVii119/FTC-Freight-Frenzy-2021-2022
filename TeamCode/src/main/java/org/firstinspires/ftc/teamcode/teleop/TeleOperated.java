package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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

    private DcMotor turretMotor;

    private Motor duckRightServo;

    private Servo depositServo;
    private Servo armServo;

    private DcMotor slideMotor;

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

    GamepadButton carouselButton;

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

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        duckRightServo = hardwareMap.get(Motor.class, "duckRightServo");

        depositServo = hardwareMap.get(Servo.class, "depositServo");

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        armServo = hardwareMap.get(Servo.class, "armServo");

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);

        // Initialize subsystems and commands
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem, intakeCommand);

        carouselSubsystem = new CarouselSubsystem(duckRightServo);

        turretSubsystem = new TurretSubsystem(turretMotor);
        turretCommand = new TurretCommand(turretSubsystem);

        slideSubsystem = new SlideSubsystem(slideMotor);
        slideCommand = new SlideCommand(slideSubsystem);

        armSubsystem = new ArmSubsystem(armServo);

        startCarouselCommand = new InstantCommand(() -> {
            if (!carouselSubsystem.isCarouselRunning) {
                carouselSubsystem.runCarousel();
            } else {
                carouselSubsystem.stopCarousel();
            }
        }, carouselSubsystem);

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
            slideSubsystem.extendToHub();
        });

        scoreCommand = new InstantCommand(() -> {
            depositSubsystem.openDeposit();
        });

        returnSlideCommand = new InstantCommand(() -> {
            slideSubsystem.returnHome();
            armSubsystem.retractArm();
            intakeSubsystem.runIntake();
        });


        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(startCarouselCommand);
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeReverseCommand);
        Button useSlideButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(useSlideCommand);
        Button scoreButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(scoreCommand);
        Button returnSlideButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(returnSlideCommand);

        // Register subsystems and set their default commands
        register(driveSubsystem, intakeSubsystem, carouselSubsystem, slideSubsystem, turretSubsystem, depositSubsystem, armSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        slideSubsystem.setDefaultCommand(slideCommand);
        turretSubsystem.setDefaultCommand(turretCommand);
        depositSubsystem.setDefaultCommand(depositCommand);
    }
}