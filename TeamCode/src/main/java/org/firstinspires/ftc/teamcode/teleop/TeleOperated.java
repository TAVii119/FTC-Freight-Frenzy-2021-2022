package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.commands.DepositCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FourBarCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name="TeleOperated")

public class TeleOperated extends CommandOpMode {

    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;

    private Motor intakeMotor;
    private Motor duckMotor;

    private Servo depositServo;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;

    // Declare subsystems and commands
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    private CarouselSubsystem carouselSubsystem;

    private DepositSubsystem depositSubsystem;
    private DepositCommand depositCommand;

    private IntakeLiftSubsystem intakeLiftSubsystem;

    private FourBarSubsystem fourBarSubsystem;
    private FourBarCommand fourBarCommand;

    // Declare instant commands, these are commands that run upon a button press
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;
    private InstantCommand liftIntakeCommand;
    private InstantCommand carouselCommand;
    private InstantCommand pushDepositCommand;

    private InstantCommand levelTopFourBarCommand;
    private InstantCommand levelMidFourBarCommand;
    private InstantCommand levelLowFourBarCommand;
    private InstantCommand moveFourBarCommand;

    Timing.Timer scoreTimer;

    // Declare gamepads
    GamepadEx driver1;
    GamepadEx driver2;

    @Override
    public void initialize() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station)
        lf = new Motor(hardwareMap, "lf");
        rf = new Motor(hardwareMap, "rf");
        lb = new Motor(hardwareMap, "lb");
        rb = new Motor(hardwareMap, "rb");
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        duckMotor = new Motor(hardwareMap, "duckMotor");

        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        duckMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Invert motors
        intakeMotor.setInverted(true);
        duckMotor.setInverted(true);

        // Get servo hardware
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");

        // Invert servos
        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);

        // Home all servos
        depositServo.setPosition(0);
        iLifterServo.setPosition(0);
        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems and commands

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem);

        carouselSubsystem = new CarouselSubsystem(duckMotor);

        intakeLiftSubsystem = new IntakeLiftSubsystem(iLifterServo);

        fourBarSubsystem = new FourBarSubsystem(gbServoLeft, gbServoRight);
        fourBarCommand = new FourBarCommand(fourBarSubsystem);

        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        // Instant commands
        intakeRunCommand = new InstantCommand(() -> {
            if (intakeMotor.get() != 0)
                intakeSubsystem.stopIntake();
            else
                intakeSubsystem.runIntake();
        }, intakeSubsystem, fourBarSubsystem, depositSubsystem);

        pushDepositCommand = new InstantCommand(() ->{
          depositSubsystem.pushDeposit();
            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            fourBarSubsystem.fourBarIntakePos();
            depositSubsystem.openDeposit();
        }, depositSubsystem, fourBarSubsystem, depositSubsystem);


        intakeReverseCommand = new InstantCommand(() -> {
            intakeSubsystem.reverseIntake();
        }, intakeSubsystem);


        liftIntakeCommand = new InstantCommand(() -> {
            if (intakeLiftSubsystem.isStraight || iLifterServo.getPosition() == 0.0)
                intakeLiftSubsystem.lifterIntakePos();
            else intakeLiftSubsystem.lifterIntakePos();
        }, intakeLiftSubsystem);


        carouselCommand = new InstantCommand(() -> {
            if (carouselSubsystem.isCarouselRunning)
                carouselSubsystem.stopCarousel();
            else carouselSubsystem.runCarousel();
        }, carouselSubsystem);

        moveFourBarCommand = new InstantCommand(() -> {
            depositSubsystem.closeDeposit();
            intakeSubsystem.reverseIntake();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            intakeSubsystem.stopIntake();
            fourBarSubsystem.fourBarTopPos();
        },  fourBarSubsystem, depositSubsystem, intakeSubsystem);

        levelTopFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.fourBarTopPos();
        },  fourBarSubsystem);


        levelMidFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.fourBarMidPos();
        },  fourBarSubsystem);


        levelLowFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.fourBarLowPos();
        },  fourBarSubsystem);


        // State the buttons that run commands
        // Using a PS4 Controller: Cross = A, Circle = B, Triangle = Y, Square = X
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);
        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(carouselCommand);
        Button liftIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(liftIntakeCommand);
        Button pushDepositButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(pushDepositCommand);

        // Buttons for the fourBar Commands(Low, Mid, Top)
        Button moveFourBarButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(moveFourBarCommand);
        Button levelLowButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(levelLowFourBarCommand);
        Button levelMidButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(levelMidFourBarCommand);
        Button levelTopButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(levelTopFourBarCommand);

        // Register subsystems and set their default commands (default commands = commands that run all the time)
        register(driveSubsystem, depositSubsystem, intakeSubsystem, carouselSubsystem, intakeLiftSubsystem, fourBarSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        fourBarSubsystem.setDefaultCommand(fourBarCommand);
    }
}