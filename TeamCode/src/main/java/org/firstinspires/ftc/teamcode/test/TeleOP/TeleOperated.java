package org.firstinspires.ftc.teamcode.test.TeleOP;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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
   private Servo rarm;
   private Servo larm;

    // Declaring subsystmes

    private DriveSubsystem driveSystem;
    private IntakeSubsystem intakeSystem;
    private ArmSubsystem armSystem;

    // Declaring commands

    private DriveCommand driveCommand;
    private IntakeCommand intakeCommand;
    private ArmCommand armCommand;

    // Declaring instant commands

    private InstantCommand level1Command;
    private InstantCommand level2Command;
    private InstantCommand level3Command;
    public InstantCommand levelIntakeCommand;

    // Insert instant command here

    private GamepadEx driver1, driver2;

    // Insert buttons here

    private Button level1, level2, level3;
    private Trigger levelIntake123;

    @Override
        public void initialize() {

        RB = new Motor(hardwareMap, "RB");
        RF = new Motor(hardwareMap, "RF");
        LB = new Motor(hardwareMap, "LB");
        LF = new Motor(hardwareMap, "LF");
        lintakeMotor = new Motor(hardwareMap, "lintakeMotor");
        rintakeMotor = new Motor(hardwareMap, "rintakeMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rarm = hardwareMap.get(Servo.class, "rarm");
        larm = hardwareMap.get(Servo.class, "larm");

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Subsystems and TEST.Commands

        // Intake subsystem and it's command
        intakeSystem = new IntakeSubsystem(lintakeMotor, rintakeMotor);
        intakeCommand = new IntakeCommand(intakeSystem, driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        // Drive subsystem and it's command
        driveSystem = new DriveSubsystem(RB, RF, LB, LF);
        driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getRightX, driver1::getLeftX);

        // Arm subsystem and it's command;
        armSystem = new ArmSubsystem(rarm, larm);
        armCommand = new ArmCommand(armSystem);

        level1Command = new InstantCommand(()-> {
            armSystem.level1();
        }, armSystem);

        level2Command = new InstantCommand(()-> {
            armSystem.level2();
        }, armSystem);

        level3Command = new InstantCommand(()-> {
            armSystem.level3();
        }, armSystem);

        // High Level, Mid Level, Low Level, Wait Level, Intaking Level

        // Buttons that run the commands
        // Insert buttons here

        level1 = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(level1Command);
        level2 = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(level2Command);
        level3 = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(level3Command);

        if (intakeSystem.intakePower > 0) {
            armSystem.levelIntake();
        }

        // Registering and default commands of our subsystems
        register(driveSystem, intakeSystem, armSystem);
        armSystem.setDefaultCommand(armCommand);
        driveSystem.setDefaultCommand(driveCommand);
        intakeSystem.setDefaultCommand(intakeCommand);
    }
}
