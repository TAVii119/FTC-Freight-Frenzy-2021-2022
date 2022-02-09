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
import org.firstinspires.ftc.teamcode.commands.Score2Command;
import org.firstinspires.ftc.teamcode.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TSESubsystem;

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
    private Servo tseServo;

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

    private ScoreCommand scoreCommand;

    private Score2Command score2Command;

    // Declare instant commands, these are commands that run upon a button press
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;
    private InstantCommand liftIntakeCommand;
    private InstantCommand carouselCommand;
    private InstantCommand pushDepositCommand;

    // Instant Commands for TSE
    private InstantCommand tsePickUpCommand;
    private InstantCommand tseWaitCommand;
    private InstantCommand tseScoreCommand;
    private InstantCommand tseMoveUpCommand;
    private InstantCommand tseMoveDownCommand;

    // Insant Commands for foruBar;
    private InstantCommand levelTopFourBarCommand;
    private InstantCommand levelMidFourBarCommand;
    private InstantCommand levelLowFourBarCommand;
    private InstantCommand moveFourBarCommand;

    Timing.Timer scoreTimer;

    Thread ScoreCommandThread;

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
        tseServo = hardwareMap.get(Servo.class, "tseServo");

        // Invert servos
        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        tseServo.setDirection(Servo.Direction.REVERSE);

        // Home all servos
        depositServo.setPosition(0.16);
        iLifterServo.setPosition(0);
        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
        tseServo.setPosition(0.02);

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems and commands

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem);

        carouselSubsystem = new CarouselSubsystem(duckMotor);

        fourBarSubsystem = new FourBarSubsystem(gbServoLeft, gbServoRight);
        fourBarCommand = new FourBarCommand(fourBarSubsystem);

        intakeLiftSubsystem = new IntakeLiftSubsystem(iLifterServo);

        tseSubsystem = new TSESubsystem(tseServo);

        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        scoreCommand = new ScoreCommand(fourBarSubsystem, depositSubsystem, intakeSubsystem, intakeLiftSubsystem);
        score2Command = new Score2Command(fourBarSubsystem, depositSubsystem, intakeSubsystem , intakeLiftSubsystem);

        ScoreCommandThread = new Thread(() -> {
            depositSubsystem.pushDeposit();
            // Wait for minerals to be ejected from deposit
            scoreTimer = new Timing.Timer(450);
            scoreTimer.start();

            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();
            fourBarSubsystem.fourBarIntakePos();
            intakeSubsystem.runIntake();
            depositSubsystem.openDeposit();

            scoreTimer = new Timing.Timer(650);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();
            intakeLiftSubsystem.lifterIntakePos();
        });
        // Instant commands
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
            if (intakeLiftSubsystem.isStraight || iLifterServo.getPosition() == 0.0)
                intakeLiftSubsystem.lifterIntakePos();
        }, intakeLiftSubsystem);

        carouselCommand = new InstantCommand(() -> {
            if (carouselSubsystem.isCarouselRunning)
                carouselSubsystem.stopCarousel();
            else
            {
                scoreTimer = new Timing.Timer(800);
                scoreTimer.start();
                while (!scoreTimer.done())
                {
                   carouselSubsystem.runCarousel();
                }
                scoreTimer.pause();
            carouselSubsystem.powerCarousel();}
        }, carouselSubsystem);

//        moveFourBarCommand = new InstantCommand(() -> {
//            depositSubsystem.closeDeposit();
//            intakeSubsystem.reverseIntake();
//            intakeLiftSubsystem.iLifterfourBarPos();
//
//            scoreTimer = new Timing.Timer(250);
//            scoreTimer.start();
//            while (!scoreTimer.done())
//            {
//                // Wait for timer to end
//            }
//            scoreTimer.pause();
//
//            fourBarSubsystem.fourBarTopPos();
//        },  fourBarSubsystem, depositSubsystem, intakeSubsystem, intakeLiftSubsystem);

//       pushDepositCommand = new InstantCommand(() ->{
//             depositSubsystem.pushDeposit();
//
//            scoreTimer = new Timing.Timer(350);
//            scoreTimer.start();
//            while (!scoreTimer.done())
//            {
//                // Wait for timer to end
//            }
//            scoreTimer.pause();
//
//            fourBarSubsystem.fourBarIntakePos();
//            intakeSubsystem.runIntake();
//            depositSubsystem.openDeposit();
//
//            scoreTimer = new Timing.Timer(900);
//            scoreTimer.start();
//            while (!scoreTimer.done())
//            {
//                // Wait for timer to end
//            }
//            scoreTimer.pause();
//
//            intakeLiftSubsystem.lifterIntakePos();
//        }, depositSubsystem, fourBarSubsystem, intakeLiftSubsystem, intakeSubsystem);

        // Insant Comamnds for the FourBar

        levelTopFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.fourBarTopPos();
        },  fourBarSubsystem);

        levelMidFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.fourBarMidPos();
        },  fourBarSubsystem);

        levelLowFourBarCommand = new InstantCommand(()-> {
            fourBarSubsystem.fourBarLowPos();
        },  fourBarSubsystem);

        // Instant Commands for TSE

        tsePickUpCommand = new InstantCommand(() -> {
            tseSubsystem.TSEPickup();
        }, tseSubsystem);

        tseWaitCommand = new InstantCommand(() ->{
            tseSubsystem.TSEWait();
        }, tseSubsystem);

        tseScoreCommand = new InstantCommand(() -> {
            tseSubsystem.initPos();
        }, tseSubsystem);

        tseMoveUpCommand = new InstantCommand(() -> {
            tseSubsystem.TSEManualControl(-0.03);
        }, tseSubsystem);

        tseMoveDownCommand = new InstantCommand(() -> {
            tseSubsystem.TSEManualControl(0.03);
        }, tseSubsystem);

        // State the buttons that run commands
        // Using a PS4 Controller: Cross = A, Circle = B, Triangle = Y, Square = X
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);
        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(carouselCommand);
        Button liftIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(liftIntakeCommand);
        Button pushDepositButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> ScoreCommandThread.start());
        Button moveFourBarButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(scoreCommand);

        // Control for driver 2;
        Button levelLowButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(levelLowFourBarCommand);
        Button levelMidButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(levelMidFourBarCommand);
        Button levelTopButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(levelTopFourBarCommand);

        Button tseScoreButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(tseScoreCommand);
        Button tsePickUpButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(tsePickUpCommand);
        Button tseWaitButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(tseWaitCommand);
        Button tseMoveUpButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(tseMoveUpCommand);
        Button tseMoveDownButton = new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(tseMoveDownCommand);

        // Register subsystems and set their default commands (default commands = commands that run all the time)
        register(driveSubsystem, depositSubsystem, intakeSubsystem, carouselSubsystem, intakeLiftSubsystem, fourBarSubsystem, tseSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}