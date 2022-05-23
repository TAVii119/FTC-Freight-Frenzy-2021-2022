package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.commands.AutomaticLiftCommand;
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

@TeleOp(name="BlueTeleOpNoReset", group = "Manual Blue")

public class BlueTeleOpNoReset extends CommandOpMode {

    boolean Pose2 = false;

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

    // Declare sensors
    private RevColorSensorV3 cupSensor;

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

    private AutomaticLiftCommand automaticLiftCommand;

    // Declare instant commands, these are commands that run upon a button press
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;
    private InstantCommand dropIntakeCommand;
    private InstantCommand liftIntakeCommand;

    // Instant Commands for TSE
    private InstantCommand tsePickUpCommand;
    private InstantCommand tseWaitCommand;
    private InstantCommand tseScoreCommand;

    private InstantCommand tseMoveUpCommand;
    private InstantCommand tseMoveDownCommand;
    private InstantCommand tseClawCloseCommand;
    private InstantCommand tseClawOpenCommand;
    private InstantCommand tseCloseCommand;

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
    public Thread tsePrepareThread;
    public Thread tseResetThread;
    public Thread tsePickUpThread;
    public Thread tseScoreThread;
    public Thread tseMoveUpThread;
    public Thread tseReleaseThread;


    Timing.Timer scoreTimer;
    boolean levelLowScore = false;
    boolean sharedHubToggle = false;

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
        iLifterServo.setPosition(0.23);
        gbServoLeft.setPosition(0.015);
        gbServoRight.setPosition(0.015);
        tseArmServo.setPosition(0.33);
        tseClawServo.setPosition(0);

        // Initialize sensors
        cupSensor = hardwareMap.get(RevColorSensorV3.class, "cupSensor");

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

        automaticLiftCommand = new AutomaticLiftCommand(depositSubsystem, fourBarSubsystem, intakeLiftSubsystem, cupSensor, intakeSubsystem);

        // Scoring Threads
        levelTopandLow = new Thread(() ->{
            if(!Pose2)
            {
                fourBarSubsystem.isMoving = true;
                intakeLiftSubsystem.lifterIntakePos();
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
                levelLowScore = true;

                if(!sharedHubToggle) {
                    fourBarSubsystem.isMoving = true;
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
                    intakeLiftSubsystem.lifterInitPos();
                }
                else {
                    fourBarSubsystem.isMoving = true;
                    depositSubsystem.closeDeposit();

                    scoreTimer = new Timing.Timer(200);
                    scoreTimer.start();
                    while (!scoreTimer.done())
                    {

                    }
                    scoreTimer.pause();

                    intakeSubsystem.reverseIntake();
                    fourBarSubsystem.fourBarIntermediate();
                    slideSubsystem.slideLowExtended();
                    fourBarSubsystem.fourBarSharedMiddlePos();

                    scoreTimer = new Timing.Timer(200);
                    scoreTimer.start();
                    while (!scoreTimer.done())
                    {

                    }
                    scoreTimer.pause();
                    intakeSubsystem.stopIntake();
                    intakeLiftSubsystem.lifterInitPos();
                }
            }
        });

        levelMidThread = new Thread(() -> {
            fourBarSubsystem.isMoving = true;
            intakeLiftSubsystem.lifterIntakePos();
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
            levelLowScore = true;

            fourBarSubsystem.isMoving = true;
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
            intakeLiftSubsystem.lifterInitPos();
        });

        scoreCommandThread = new Thread(() -> {
            if(levelLowScore)
            {
                intakeLiftSubsystem.lifterIntakePos();
                fourBarSubsystem.isMoving = true;
                depositSubsystem.openDeposit();

                scoreTimer = new Timing.Timer(200);
                scoreTimer.start();
                while (!scoreTimer.done())
                {

                }
                scoreTimer.pause();

                fourBarSubsystem.fourBarIntermediateScore();

                scoreTimer = new Timing.Timer(400);
                scoreTimer.start();
                while (!scoreTimer.done())
                {

                }
                scoreTimer.pause();

                slideSubsystem.slideHome();

                fourBarSubsystem.fourBarIntake();
                depositSubsystem.depositIntermediate();

                scoreTimer = new Timing.Timer(150);
                scoreTimer.start();
                while (!scoreTimer.done())
                {

                }
                intakeSubsystem.runIntake();
                fourBarSubsystem.isMoving = false;
                levelLowScore = false;
            }
            else {
                intakeLiftSubsystem.lifterIntakePos();
                fourBarSubsystem.isMoving = true;
                depositSubsystem.openDeposit();

                scoreTimer = new Timing.Timer(200);
                scoreTimer.start();
                while (!scoreTimer.done()) {

                }
                scoreTimer.pause();

                fourBarSubsystem.fourBarIntermediateScore();

                scoreTimer = new Timing.Timer(500);
                scoreTimer.start();
                while (!scoreTimer.done()) {

                }
                scoreTimer.pause();

                slideSubsystem.slideHome();

                fourBarSubsystem.fourBarIntake();
                depositSubsystem.depositIntermediate();

                scoreTimer = new Timing.Timer(150);
                scoreTimer.start();
                while (!scoreTimer.done()) {

                }
                intakeSubsystem.runIntake();
                fourBarSubsystem.isMoving = false;
            }
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

        dropIntakeCommand = new InstantCommand(() -> {
            intakeLiftSubsystem.lifterIntakePos();
        }, intakeLiftSubsystem);

        liftIntakeCommand = new InstantCommand(() -> {
            if(intakeLiftSubsystem.isLifted) {
                intakeLiftSubsystem.lifterIntakePos();
                intakeLiftSubsystem.isLifted = false;
            }
            else {
                intakeLiftSubsystem.lifterInitPos();
                intakeLiftSubsystem.isLifted = true;
            }
        }, intakeLiftSubsystem);


        // Threads for the TSE

        tseCloseCommand = new InstantCommand(() -> {
            tseClawServo.setPosition(0);
        }, tseSubsystem);

        tsePickUpThread = new Thread(() -> {
            tseSubsystem.tsePickup();
            tseSubsystem.tseClawRelease();
        });

        tseScoreThread = new Thread(() ->
        {
            tseSubsystem.tseClawGrip();
            intakeSubsystem.stopIntake();
            iLifterServo.setPosition(0);

            scoreTimer = new Timing.Timer(300);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            tseSubsystem.tseScore();
        });

        tseReleaseThread = new Thread(() ->{
            tseSubsystem.tseClawRelease();
        });

        tseMoveUpCommand = new InstantCommand(() -> {
            tseSubsystem.TSEManualControl(-0.05);
        }, tseSubsystem);

        tseMoveDownCommand = new InstantCommand(() -> {
            tseSubsystem.TSEManualControl(0.05);
        }, tseSubsystem);

        tseClawOpenCommand = new InstantCommand(()-> {
            tseSubsystem.TSEClawManualControl(0.05);
        }, tseSubsystem);

        tseClawCloseCommand = new InstantCommand(() -> {
            tseSubsystem.TSEClawManualControl(-0.05);
        }, tseSubsystem);

        tseResetThread = new Thread(() -> {
            intakeLiftSubsystem.lifterIntakePos();
            tseSubsystem.tseWait();
        });

        changeLevelCommand = new InstantCommand(() -> {
            if(!Pose2)
                Pose2 = true;
            else
                Pose2 = false;
        });

        Thread duckMidThread = new Thread(() -> {
            fourBarSubsystem.isMoving = true;
            intakeLiftSubsystem.lifterIntakePos();
            depositSubsystem.closeDeposit();

            scoreTimer = new Timing.Timer(600);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            fourBarSubsystem.fourBarIntermediateScore();

            slideSubsystem.slideMid();
            fourBarSubsystem.fourBarIntermediateDuck();
        });


        // State the buttons that run commands
        // Using a PS4 Controller: Cross = A, Circle = B, Triangle = Y, Square = X
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);
        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(() -> carouselThread.start());
        Button liftIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(liftIntakeCommand);
        Button dropIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(dropIntakeCommand);
        Button switchLow = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(changeLevelCommand);
        Button switchShared = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            sharedHubToggle = !sharedHubToggle;
            Pose2 = true;
        });


        // Scoring Command Threads
        Button useSlideButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> levelTopandLow.start());
        Button scoreButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> scoreCommandThread.start());

        // Control for driver 2
        Button levelMidButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> levelMidThread.start());
        Button levelLowButton = new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> duckMidThread.start());

        Button tseClawCloseButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(tseCloseCommand);
        Button tsePickupButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> tsePickUpThread.start());
        Button tseScoreButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> tseScoreThread.start());
        Button tseReleaseButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(() -> tseReleaseThread.start());
        Button tseManualUp = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(tseMoveUpCommand);
        Button tseManualDown = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(tseMoveDownCommand);
        Button tseReset = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(() -> tseResetThread.start());
        Button liftIntakeDriver2Button = new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).whenPressed(liftIntakeCommand);

        // Register subsystems and set their default commands (default commands = commands that run all the time)
        register(driveSubsystem, depositSubsystem, intakeSubsystem, carouselSubsystem, intakeLiftSubsystem, fourBarSubsystem, tseSubsystem, slideSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}