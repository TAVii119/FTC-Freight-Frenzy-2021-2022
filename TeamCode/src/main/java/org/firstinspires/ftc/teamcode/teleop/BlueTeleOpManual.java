package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TSESubsystem;

@TeleOp(name="BlueTeleOpManual", group = "Manual Red")
public class BlueTeleOpManual extends CommandOpMode {

    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;

    private Motor intakeMotor;
    private Motor duckMotor;

    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo deposit1;
    private Servo iLifterServo;
    private Servo gbServoRight;
    private Servo gbServoLeft;

    private Servo tseArmServo;
    private Servo tseClawServo;


    boolean Pose2 = false;

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

    private TSESubsystem tseSubsystem;

    private SlideSubsystem slideSubsystem;

    // Declare instant commands, these are commands that run upon a button press
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;
    private InstantCommand liftIntakeCommand;

    // Instant Commands for TSE
    private InstantCommand tsePickUpCommand;
    private InstantCommand tseWaitCommand;
    private InstantCommand tseScoreCommand;
    private InstantCommand tseMoveUpCommand;
    private InstantCommand tseMoveDownCommand;

    // Insant Commands for fourBar;
    private InstantCommand levelTopFourBarCommand;
    private InstantCommand levelMidFourBarCommand;
    private InstantCommand levelLowFourBarCommand;

    private InstantCommand changeLevelCommand;


    // Threads used for Scoring

    public Thread levelTopandLow;
    public Thread scoreCommandThread;
    public Thread levelMidThread;
    public Thread levelLowThread;
    public Thread carouselThread;
    public Thread tseScoreThread;
    public Thread tseReleaseThread;
    public Thread tsePickUpThread;
    public Thread tseMoveUpThread;
    public Thread tseMoveDown;
    public Thread tseWaitThread;


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
        leftSlideMotor = new Motor(hardwareMap, "leftSlideMotor");
        rightSlideMotor = new Motor(hardwareMap, "rightSlideMotor");

        // Set zero power behavior
        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        duckMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        leftSlideMotor.resetEncoder();
        rightSlideMotor.resetEncoder();
        duckMotor.resetEncoder();

        // Invert motors
        intakeMotor.setInverted(true);
        rightSlideMotor.setInverted(true);

        // Get servo hardware
        deposit1 = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        tseArmServo = hardwareMap.get(Servo.class, "tseArmServo");
        tseClawServo = hardwareMap.get(Servo.class, "tseClawServo");

        // Invert servos
        deposit1.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        tseArmServo.setDirection(Servo.Direction.REVERSE);

        // Home all servos
        deposit1.setPosition(0.12);
        iLifterServo.setPosition(0.27);
        gbServoLeft.setPosition(0.022);
        gbServoRight.setPosition(0.022);
        tseArmServo.setPosition(0.16);
        tseClawServo.setPosition(0);

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems and commands

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(deposit1);
        depositCommand = new DepositCommand(depositSubsystem);

        carouselSubsystem = new CarouselSubsystem(duckMotor);

        fourBarSubsystem = new FourBarSubsystem(gbServoLeft, gbServoRight);
        fourBarCommand = new FourBarCommand(fourBarSubsystem);

        intakeLiftSubsystem = new IntakeLiftSubsystem(iLifterServo);

        tseSubsystem = new TSESubsystem(tseArmServo, tseClawServo);

        slideSubsystem = new SlideSubsystem(rightSlideMotor, leftSlideMotor);

        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        // Scoring Threads
        levelTopandLow = new Thread(() ->{
            if(!Pose2)
            {
                depositSubsystem.closeDeposit();

                scoreTimer = new Timing.Timer(200);
                scoreTimer.start();
                while (!scoreTimer.done()) {

                }
                scoreTimer.pause();

                intakeSubsystem.reverseIntake();
                fourBarSubsystem.fourBarIntermediate();
                slideSubsystem.slideTop();
                fourBarSubsystem.fourBarTop();

                scoreTimer = new Timing.Timer(200);
                scoreTimer.start();
                while (!scoreTimer.done()) {

                }
                scoreTimer.pause();

                intakeSubsystem.stopIntake();
            }
            else
            {
                depositSubsystem.closeDeposit();

                scoreTimer = new Timing.Timer(200);
                scoreTimer.start();
                while (!scoreTimer.done())
                {

                }
                scoreTimer.pause();

                intakeSubsystem.reverseIntake();
                fourBarSubsystem.fourBarIntermediate();
                slideSubsystem.slideLow();
                fourBarSubsystem.fourBarLow();

                scoreTimer = new Timing.Timer(200);
                scoreTimer.start();
                while (!scoreTimer.done())
                {

                }
                scoreTimer.pause();
                intakeSubsystem.stopIntake();
            }
        });

        levelMidThread = new Thread(() -> {
            depositSubsystem.closeDeposit();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeSubsystem.reverseIntake();
            fourBarSubsystem.fourBarIntermediate();
            slideSubsystem.slideMid();
            fourBarSubsystem.fourBarMid();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeSubsystem.stopIntake();
        });

        levelLowThread = new Thread(() -> {
            depositSubsystem.closeDeposit();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeSubsystem.reverseIntake();
            fourBarSubsystem.fourBarIntermediate();
            slideSubsystem.slideLow();
            fourBarSubsystem.fourBarLow();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            intakeSubsystem.stopIntake();
        });

        scoreCommandThread = new Thread(() -> {
            slideSubsystem.slideMoving = true;
            depositSubsystem.openDeposit();

            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            fourBarSubsystem.fourBarIntermediate();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            slideSubsystem.slideHome();

            scoreTimer = new Timing.Timer(125);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();

            fourBarSubsystem.fourBarIntake();
            intakeSubsystem.runIntake();
            depositSubsystem.depositIntermediate();

            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while(!scoreTimer.done())
            {

            }
            slideSubsystem.slideMoving = false;
        });

        carouselThread = new Thread(() -> {
            carouselSubsystem.runCarousel();
        });

        // Instant Commands

        intakeRunCommand = new InstantCommand(() -> {
            if (intakeMotor.get() != 0)
                intakeSubsystem.stopIntake();
            else
                intakeSubsystem.runIntake();
        }, intakeSubsystem, fourBarSubsystem, depositSubsystem);

        intakeReverseCommand = new InstantCommand(() -> {
            intakeSubsystem.reverseIntake();
        }, intakeSubsystem);

        liftIntakeCommand = new InstantCommand(() -> {
                intakeLiftSubsystem.lifterIntakePos();
        }, intakeLiftSubsystem);


        // Threads for the TSE

        tsePickUpThread = new Thread(() -> {
            tseSubsystem.tsePickup();
            tseSubsystem.tseClawRelease();
        });

        tseScoreThread = new Thread(() ->
        {
            tseSubsystem.tseClawGrip();
            iLifterServo.setPosition(0);
            scoreTimer = new Timing.Timer(400);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            tseSubsystem.tseScore();
        });

        tseReleaseThread = new Thread(() ->{
            tseSubsystem.tseClawRelease();
            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            tseSubsystem.tseWait();
        });

        tseMoveUpThread = new Thread(() -> {
                tseSubsystem.TSEManualControl(-0.05);
        });

        tseMoveDownCommand = new InstantCommand(() -> {
            tseSubsystem.TSEManualControl(0.05);
        }, tseSubsystem);

        tseWaitThread = new Thread(() ->{
            tseSubsystem.tseWait();
        });

        changeLevelCommand = new InstantCommand(() -> {
            if(!Pose2)
                Pose2 = true;
            else
                Pose2 = false;
        });



        // State the buttons that run commands
        // Using a PS4 Controller: Cross = A, Circle = B, Triangle = Y, Square = X
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);
        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(() -> carouselThread.start());
        Button liftIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(liftIntakeCommand);
        Button SwitchShared = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(changeLevelCommand);

        // Scoring Command Threads
        Button useSlideButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> levelTopandLow.start());
        Button scoreButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> scoreCommandThread.start());

        // Control for driver 2
        Button levelMidButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> levelMidThread.start());

        Button tsePickupButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> tsePickUpThread.start());
        Button tseScoreButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> tseScoreThread.start());
        Button tseReleaseButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(() -> tseReleaseThread.start());
        Button tseManualUp = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(() -> tseMoveUpThread.start());
        Button tseManualDown = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(tseMoveDownCommand);

        // Register subsystems and set their default commands (default commands = commands that run all the time)
        register(driveSubsystem, depositSubsystem, intakeSubsystem, carouselSubsystem, intakeLiftSubsystem, fourBarSubsystem, tseSubsystem, slideSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}