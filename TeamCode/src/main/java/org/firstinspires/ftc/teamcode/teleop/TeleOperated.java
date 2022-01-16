package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name="TeleOperated")

public class TeleOperated extends CommandOpMode {
    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;

    // Declare commands and subsystems
    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    GamepadEx driver1;
    GamepadEx driver2;

    @Override
    public void initialize() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station).
        lf = new Motor(hardwareMap, "lf");
        lb = new Motor(hardwareMap, "lb");
        rf = new Motor(hardwareMap, "rf");
        rb = new Motor(hardwareMap, "rb");

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems and commands
        driveSubsystem = new DriveSubsystem(lf, lb, rf, rb);
        driveCommand = new DriveCommand(driveSubsystem, () -> driver1.getLeftY(), () -> driver1.getLeftX(), () -> driver1.getRightX());

        // Register subsystems and set their default commands
        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
