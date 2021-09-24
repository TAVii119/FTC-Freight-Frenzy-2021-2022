package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

    /*
    This is our TeleOperated class. We control the robot using two DualShock 4 controllers.

    We use subsystems and commands. The goal of this is to improve the initial programming experience
    for new members as well as greatly enhance the efficiency of code for veterans.

    Gamepad Controls:

    Gamepad1:

    Left Joystick : Drive/Reverse the robot
    Right Joystick : Rotates robot
    Left Trigger : Intake freight (game elements)
    Right Trigger : Outtake freight (game elements)
    Left Bumper : N/A
    Right Bumper : N/A
    CROSS : N/A
    TRIANGLE : N/A
    SQUARE : N/A
    CIRCLE : N/A
    */

@TeleOp(name="TeleOperated")
public class TeleOperated extends CommandOpMode {

    // Declaring Motors, Servos and Sensors

    private Motor intakeMotor;
    private MotorGroup chassisLeftSideMotors, chassisRightSideMotors;

    // Declaring subsystmes

    private IntakeSubsystem intakeSubsystem;
    private DriveSubsystem driveSubsystem;

    // Declaring commands

    private IntakeCommand intakeCommand;
    private DriveCommand driveCommand;

    // Declaring instant commands

    // Insert instant command here

    private GamepadEx driver1, driver2;

    // Insert buttons here

    @Override
    public void initialize() {
        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_435);

        chassisLeftSideMotors = new MotorGroup(
                new Motor(hardwareMap, "leftMotor1", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "leftMotor2", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "leftMotor3", Motor.GoBILDA.RPM_435)
        );

        chassisRightSideMotors = new MotorGroup(
                new Motor(hardwareMap, "rightMotor1", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "rightMotor2", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "rightMotor3", Motor.GoBILDA.RPM_435)
        );

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Subsystems and Commands

        // Intake subsystem and it's command
        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem, driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        // Drive subsystem and it's command
        driveSubsystem = new DriveSubsystem(chassisLeftSideMotors, chassisRightSideMotors);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftY, driver1::getRightX);

        // Buttons that run the commands
        // Insert buttons here

        // Registering and default commands of our subsystems
        register(intakeSubsystem, driveSubsystem);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
