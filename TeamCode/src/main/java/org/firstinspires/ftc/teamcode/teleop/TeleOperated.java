package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name="TeleOperated")
public class TeleOperated extends CommandOpMode {

    // Declaring Motors, Servos and Sensors

    private Motor intakeMotor;

    // Declaring subsystmes

    private IntakeSubsystem intakeSubsystem;

    // Declaring commands

    private IntakeCommand intakeCommand;

    // Declaring instant commands

    private InstantCommand stopIntakeCommand;

    private GamepadEx driver1, driver2;
    private Button intakeButton, stopIntakeButton;

    @Override
    public void initialize() {
        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_435);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Subsystems and Commands

        // Intake subsystem and it's commands
        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);
        stopIntakeCommand = new InstantCommand(()-> {
            intakeSubsystem.stop();
        }, intakeSubsystem);

        // Buttons that run the commands

        intakeButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeCommand);
        stopIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(stopIntakeCommand);

        // Registering and default commands of our subsystems
        register(intakeSubsystem);
        intakeSubsystem.setDefaultCommand(intakeCommand);
    }
}
