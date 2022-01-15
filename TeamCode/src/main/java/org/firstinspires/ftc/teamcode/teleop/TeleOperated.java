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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DepositCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FourBarCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOperated", group="Test")

public class TeleOperated extends CommandOpMode {
    int seconds = 1;
    // Declare Motors and Servos
    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;
    private Motor intakeMotor;
    private DcMotor turretMotor;

    private Servo depositServo;
    private DcMotor slideMotor;
    private Servo fourBarMotor;

    // Declare commands and subsystems
    private DriveCommand driveCommand;
    private DriveSubsystem driveSubsystem;

    private FourBarCommand fourBarCommand;
    private FourBarSubsystem fourBarSubsystem;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    private DepositCommand depositCommand;
    private DepositSubsystem depositSubsystem;

    private SlideSubsystem slideSubsystem;

    private TurretSubsystem turretSubsystem;

    private InstantCommand levelTopFourBarCommand;
    private InstantCommand levelMidFourBarCommand;
    private InstantCommand levelLowFourBarCommand;
    private InstantCommand levelWaitFourBarCommand;
    private InstantCommand moveDepositCommand;
    private InstantCommand pushDepositCommand;
    private InstantCommand moveFourBarCommand;
    private InstantCommand topLevelCommand;
    private InstantCommand returnIntakeCommand;

    GamepadEx driver1;
    GamepadEx driver2;

    public static boolean intakeTimerDone = false;



    @Override
    public void initialize() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station).
        leftFront = new Motor(hardwareMap, "leftFront");
        leftBack = new Motor(hardwareMap, "leftBack");
        rightFront = new Motor(hardwareMap, "rightFront");
        rightBack = new Motor(hardwareMap, "rightBack");

        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        fourBarMotor = hardwareMap.get(Servo.class, "fourBarMotor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        depositServo = hardwareMap.get(Servo.class, "depositServo");

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems and commands
        intakeSubsystem = new IntakeSubsystem(intakeMotor, fourBarSubsystem);
        intakeCommand = new IntakeCommand(intakeSubsystem,() -> driver1.getTrigger(RIGHT_TRIGGER), () -> driver1.getTrigger(LEFT_TRIGGER));

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem, intakeCommand);

        slideSubsystem = new SlideSubsystem(slideMotor);

        turretSubsystem = new TurretSubsystem(turretMotor);


        fourBarSubsystem = new FourBarSubsystem(fourBarMotor);
        fourBarCommand = new FourBarCommand(fourBarSubsystem, slideSubsystem, intakeCommand);

        driveSubsystem = new DriveSubsystem(rightBack, leftBack, rightFront, leftFront);
        driveCommand = new DriveCommand(driveSubsystem, () -> driver1.getLeftX(), () -> driver1.getLeftY(), () -> driver1.getRightX());
        // Instant commands to control the four bar and deposit mechanisms
        // Four bar levels: {0.02, 0.06, 0.58, 0.70, 0.79} {INTAKE, HOVER, TOP GOAL, MID GOAL, LOW GOAL}

        topLevelCommand = new InstantCommand(()-> {
            depositSubsystem.closeDeposit();
            turretSubsystem.rotateToShippingHub(0);
            fourBarSubsystem.moveBarTop();
            slideSubsystem.extendSlideToScore(0);
        }, turretSubsystem, slideSubsystem, fourBarSubsystem, depositSubsystem);

        returnIntakeCommand = new InstantCommand(() -> {
            depositSubsystem.pushDeposit();
            slideSubsystem.bringSlideBack();
            turretSubsystem.returnToIntake();
            fourBarSubsystem.setLevelIntake();
        }, turretSubsystem, slideSubsystem, fourBarSubsystem, depositSubsystem);

        moveFourBarCommand = new InstantCommand(()-> {
            sleep(15);
            depositSubsystem.closeDeposit();
        }, fourBarSubsystem);

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
        Button topLevelButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(topLevelCommand);
        Button returnIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(returnIntakeCommand);
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
