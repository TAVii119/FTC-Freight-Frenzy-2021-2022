package org.firstinspires.ftc.teamcode.teleop;

import android.transition.Slide;

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

import org.firstinspires.ftc.teamcode.commands.CarouselCommand;
import org.firstinspires.ftc.teamcode.commands.DepositCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.time.Instant;

@TeleOp(name="TeleOperated")

public class TeleOperated extends CommandOpMode {
    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;

    private Motor intakeMotor;

    private DcMotor turretMotor;

    private CRServo duckLeftServo;
    private CRServo duckRightServo;

    private Servo depositServo;

    private DcMotor slideMotor;

    // Declare commands and subsystems
    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    private CarouselSubsystem carouselSubsystem;
    private CarouselCommand carouselCommand;

    private DepositSubsystem depositSubsystem;
    private DepositCommand depositCommand;

    private TurretSubsystem turretSubsystem;
    private TurretCommand turretCommand;

    private SlideSubsystem slideSubsystem;
    private SlideCommand slideCommand;

    private InstantCommand startCarouselCommand;
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;

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

        duckLeftServo = hardwareMap.get(CRServo.class, "duckLeftServo");
        duckRightServo = hardwareMap.get(CRServo.class, "duckRightServo");

        depositServo = hardwareMap.get(Servo.class, "depositServo");

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);

        // Initialize subsystems and commands
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem, intakeCommand);

        carouselSubsystem = new CarouselSubsystem(duckLeftServo, duckRightServo);
        carouselCommand = new CarouselCommand(carouselSubsystem);

        turretSubsystem = new TurretSubsystem(turretMotor);
        turretCommand = new TurretCommand(turretSubsystem);

        slideSubsystem = new SlideSubsystem(slideMotor);
        slideCommand = new SlideCommand(slideSubsystem);

        startCarouselCommand = new InstantCommand(() -> {
            if (!carouselSubsystem.carouselRunning) {
                carouselSubsystem.startCarousel();
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


        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(startCarouselCommand);
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);

        // Register subsystems and set their default commands
        register(driveSubsystem, intakeSubsystem, carouselSubsystem, slideSubsystem, turretSubsystem, depositSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        carouselSubsystem.setDefaultCommand(carouselCommand);
        slideSubsystem.setDefaultCommand(slideCommand);
        turretSubsystem.setDefaultCommand(turretCommand);
        depositSubsystem.setDefaultCommand(depositCommand);
    }
}
