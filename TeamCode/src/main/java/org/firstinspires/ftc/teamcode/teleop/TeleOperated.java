package org.firstinspires.ftc.teamcode.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.Timing;
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

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOperated", group="Test")

public class TeleOperated extends CommandOpMode {
    int seconds = 2;
    // Declare Motors and Servos
    private Motor l1; // l1, l2, l3 ; r1, r2, r3 - Chassis motors, l = left, r = right,
    private Motor l2; // numbered from front to back
    private Motor l3;
    private Motor r1;
    private Motor r2;
    private Motor r3;
    private MotorGroup leftSide;
    private MotorGroup rightSide;
    private Motor intakeMotor;

    private Servo gbServoRight;
    private Servo gbServoLeft;
    private Servo depositServo;
    private Servo iLifterServo;

    // Declare commands and subsystems
    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    private FourBarCommand fourBarCommand;
    private FourBarSubsystem fourBarSubsystem;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    private DepositCommand depositCommand;
    private DepositSubsystem depositSubsystem;

    private InstantCommand levelTopFourBarCommand;
    private InstantCommand levelMidFourBarCommand;
    private InstantCommand levelLowFourBarCommand;
    private InstantCommand levelWaitFourBarCommand;
    private InstantCommand moveDepositCommand;
    private InstantCommand pushDepositCommand;
    private InstantCommand moveFourBarCommand;

    GamepadEx driver1;
    GamepadEx driver2;

    public static Timing.Timer intakeTimer;
    public static boolean intakeTimerDone = false;

    @Override
    public void initialize() {
        intakeTimer = new Timing.Timer(seconds, TimeUnit.SECONDS);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station).
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
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");

        // Reverse directions and set initial position for servos
        iLifterServo.setDirection(Servo.Direction.REVERSE);
        iLifterServo.setPosition(0.19);

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems and commands
        intakeSubsystem = new IntakeSubsystem(intakeMotor, fourBarSubsystem);
        intakeCommand = new IntakeCommand(intakeSubsystem,() -> driver1.getTrigger(RIGHT_TRIGGER), () -> driver1.getTrigger(LEFT_TRIGGER));

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem, intakeCommand);

        fourBarSubsystem = new FourBarSubsystem(gbServoLeft, gbServoRight);
        fourBarCommand = new FourBarCommand(fourBarSubsystem, intakeCommand);

        driveSubsystem = new DriveSubsystem(leftSide, rightSide);
        driveCommand = new DriveCommand(driveSubsystem, () -> driver1.getLeftY(), () -> driver1.getRightX());
        
        // Instant commands to control the four bar and deposit mechanisms
        // Four bar levels: {0.02, 0.06, 0.58, 0.70, 0.79} {INTAKE, HOVER, TOP GOAL, MID GOAL, LOW GOAL}

        moveFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.setLevel();
            sleep(15);
            depositSubsystem.closeDeposit();
        }, fourBarSubsystem);

        levelTopFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.setDesiredLevel(2);
        }, fourBarSubsystem, depositSubsystem);

        levelMidFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.setDesiredLevel(3);
        }, fourBarSubsystem, depositSubsystem, intakeSubsystem);

        levelLowFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.setDesiredLevel(4);
        }, fourBarSubsystem, depositSubsystem);

        levelWaitFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.setLevelWait();
            depositSubsystem.openDeposit();
        }, fourBarSubsystem, depositSubsystem);

        moveDepositCommand = new InstantCommand(() -> {
            if (depositSubsystem.depositOpen)
                depositSubsystem.closeDeposit();
            else depositSubsystem.openDeposit();

        }, depositSubsystem);

        pushDepositCommand = new InstantCommand(() -> {
            depositSubsystem.pushDeposit();
        }, depositSubsystem);

        // Declare buttons to run specific commands
        Button depositPushButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(pushDepositCommand);
        Button levelWaitButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(levelWaitFourBarCommand);
        Button moveDepositButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(moveDepositCommand);
        Button moveFourBarButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(moveFourBarCommand);

        Button levelLowButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(levelLowFourBarCommand);
        Button levelMidButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(levelMidFourBarCommand);
        Button levelTopButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(levelTopFourBarCommand);

        // Register subsystems and set their default commands
        register(fourBarSubsystem, driveSubsystem, intakeSubsystem, depositSubsystem);
        depositSubsystem.setDefaultCommand(depositCommand);
        fourBarSubsystem.setDefaultCommand(fourBarCommand);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
    }
}
