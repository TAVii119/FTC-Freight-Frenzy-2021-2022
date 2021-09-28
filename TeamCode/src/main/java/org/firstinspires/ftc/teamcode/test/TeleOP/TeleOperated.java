package org.firstinspires.ftc.teamcode.test.TeleOP;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.test.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.test.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.test.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.test.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.test.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.test.Subsystems.IntakeSubsystem;
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

   private Motor LF;
   private Motor RF;
   private Motor LB;
   private Motor RB;
   private Motor lintakeMotor;
   private Motor rintakeMotor;
   private Servo arm;


    // Declaring subsystmes
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    // Declaring commands

    private DriveCommand driveCommand;
    private IntakeCommand intakeCommand;
     private ArmCommand armCommand;
    // Declaring instant commands

    // Insert instant command here

    private GamepadEx driver1;




    // Insert buttons here

    @Override
    public void initialize() {

        LF = hardwareMap.get(Motor.class, "LF");
        RF = hardwareMap.get(Motor.class, "RF");
        LB = hardwareMap.get(Motor.class, "LB");
        RB = hardwareMap.get(Motor.class, "RB");
        lintakeMotor = new Motor(hardwareMap, "lintakeMotor");
        rintakeMotor = new Motor(hardwareMap, "rintakeMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm = hardwareMap.get(Servo.class, "Arm")
        ;

        driver1 = new GamepadEx(gamepad1);

        // Subsystems and TEST.Commands

        // Intake subsystem and it's command
        intakeSubsystem = new IntakeSubsystem(lintakeMotor, rintakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem, driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        // Drive subsystem and it's command
        driveSubsystem = new DriveSubsystem(RB, RF, LB, LF);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftY, driver1::getRightX, driver1::getLeftX);

        // Arm subsystem and it's command;
        armSubsystem = new ArmSubsystem(arm);
        // Buttons that run the commands
        // Insert buttons here

        // Registering and default commands of our subsystems
        register(driveSubsystem, intakeSubsystem, armSubsystem);
        armSubsystem.setDefaultCommand(armCommand);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
    }
}
