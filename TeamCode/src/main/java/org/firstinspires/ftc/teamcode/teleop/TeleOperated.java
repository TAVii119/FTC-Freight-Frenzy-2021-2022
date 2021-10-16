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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DepositCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FourBarCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name="TeleOperated", group="Test")

public class TeleOperated extends CommandOpMode {

    // Declare OpMode members.
    private Motor l1; //l1, l2, l3 ; r1, r2, r3
    private Motor l2;
    private Motor l3;
    private Motor r1;
    private Motor r2; //l1, l2, l3 ; r1, r2, r3
    private Motor r3;
    private MotorGroup rightSide;
    private MotorGroup leftSide;
    private Motor intakeMotor;
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo depositServo;

    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    private FourBarCommand fourBarCommand;
    private FourBarSubsystem fourBarSubsystem;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    private DepositCommand depositCommand;
    private DepositSubsystem depositSubsystem;


    private InstantCommand levelTopFourBar;
    private InstantCommand levelMidFourBar;
    private InstantCommand levelLowFourBar;
    private InstantCommand levelWaitFourBar;

    private InstantCommand closeDeposit;
    private InstantCommand moveDeposit;
    private InstantCommand pushDeposit;

    private Button levelTopButton, levelMidButton, levelLowButton, levelWaitButton;
    private Button depositPushButton, depositClose, moveDepositButton;

    GamepadEx driver1;
    GamepadEx driver2;

    @Override
    public void initialize() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        l1 = new Motor(hardwareMap, "l1");
        l2 = new Motor(hardwareMap, "l2");
        l3 = new Motor(hardwareMap, "l3");
        r1 = new Motor(hardwareMap, "r1");
        r2 = new Motor(hardwareMap, "r2");
        r3 = new Motor(hardwareMap, "r3");
        rightSide = new MotorGroup(r1, r2, r3);
        leftSide = new MotorGroup(l1, l2, l3);
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        gbServoLeft= hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        depositServo = hardwareMap.get(Servo.class, "depositServo");

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem, driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        // intakeCommand = new IntakeCommand(intakeSubsystem, gamepad1.right_trigger, gamepad1.left_trigger);

        // When the intake is active, the fourbar moves to the intake position.
        if(intakeCommand.intake >0.1){
            fourBarSubsystem.setLevel(0);
            depositSubsystem.openDeposit();
        }

        driveSubsystem = new DriveSubsystem(leftSide, rightSide);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftY, driver1::getRightX);

        fourBarSubsystem = new FourBarSubsystem(gbServoLeft, gbServoRight);
        fourBarCommand = new FourBarCommand(fourBarSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem);

        levelTopFourBar = new InstantCommand(()-> {
            fourBarSubsystem.setLevel(2);
        }, fourBarSubsystem);

        levelMidFourBar = new InstantCommand(()-> {
            fourBarSubsystem.setLevel(3);
        }, fourBarSubsystem);

        levelLowFourBar = new InstantCommand(()-> {
            fourBarSubsystem.setLevel(4);
        }, fourBarSubsystem);
        levelWaitFourBar = new InstantCommand(()-> {
            fourBarSubsystem.setLevel(1);
        }, fourBarSubsystem);

        moveDeposit = new InstantCommand(() -> {
            if(depositSubsystem.depositOpen)
                depositSubsystem.closeDeposit();
            else depositSubsystem.openDeposit();

        }, depositSubsystem);

        pushDeposit = new InstantCommand(() -> {
            depositSubsystem.pushDeposit();
        }, depositSubsystem);

        depositPushButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(pushDeposit);

        levelLowButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(levelLowFourBar);
        levelMidButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(levelMidFourBar);
        levelTopButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(levelTopFourBar);
        levelWaitButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(levelWaitFourBar);
        moveDepositButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(moveDeposit);

        register(fourBarSubsystem, driveSubsystem, intakeSubsystem, depositSubsystem);
        depositSubsystem.setDefaultCommand(depositCommand);
        fourBarSubsystem.setDefaultCommand(fourBarCommand);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
    }
}
