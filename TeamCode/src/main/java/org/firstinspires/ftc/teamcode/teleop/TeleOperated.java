package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name="TeleOperated")

public class TeleOperated extends CommandOpMode {
    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;
    private Motor intakeMotor;

    // Declare commands and subsystems
    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

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

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);

        // Initialize subsystems and commands
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem, driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        // Register subsystems and set their default commands
        register(driveSubsystem, intakeSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
    }
}
