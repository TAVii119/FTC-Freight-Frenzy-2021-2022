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
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="TeleOperated")

public class TeleOperated extends CommandOpMode {

    // Declare Motors and Servos
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;

    private Motor intakeMotor;
    private Motor slideMotor;
    private Motor duckMotor;

    private Servo turretServo;
    private Servo depositServo;
    private Servo gbServoLeft;
    private Servo gbServoRight;
    private Servo iLifterServo;

    // Declare subsystems and commands
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    private CarouselSubsystem carouselSubsystem;

    private DepositSubsystem depositSubsystem;
    private DepositCommand depositCommand;

    private TurretSubsystem turretSubsystem;
    private TurretCommand turretCommand;

    private SlideSubsystem slideSubsystem;
    private SlideCommand slideCommand;

    private ArmSubsystem armSubsystem;

    private IntakeLiftSubsystem intakeLiftSubsystem;

    // Declare instant commands, these are commands that run upon a button press
    private InstantCommand intakeRunCommand;
    private InstantCommand intakeReverseCommand;
    private InstantCommand useSlideCommand;
    private InstantCommand returnSlideCommand;
    private InstantCommand liftIntakeCommand;
    private InstantCommand carouselCommand;
    private InstantCommand moveTurretLeft;
    private InstantCommand moveTurretRight;

    Timing.Timer scoreTimer;

    // Declare gamepads
    GamepadEx driver1;

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
        slideMotor = new Motor(hardwareMap, "slideMotor");
        duckMotor = new Motor(hardwareMap, "duckMotor");

        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        duckMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Invert motors
        intakeMotor.setInverted(true);
        slideMotor.setInverted(true);
        duckMotor.setInverted(true);

        // Get servo hardware
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        // Invert servos
        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        turretServo.setDirection(Servo.Direction.REVERSE);

        // Home all servos
        turretServo.setPosition(0);
        depositServo.setPosition(0);
        gbServoLeft.setPosition(0.03);
        gbServoRight.setPosition(0.03);
        iLifterServo.setPosition(0);

        // Assign gamepads to drivers
        driver1 = new GamepadEx(gamepad1);

        // Initialize subsystems and commands
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        depositSubsystem = new DepositSubsystem(depositServo);
        depositCommand = new DepositCommand(depositSubsystem);

        turretSubsystem = new TurretSubsystem(turretServo);
        turretCommand = new TurretCommand(turretSubsystem);

        slideSubsystem = new SlideSubsystem(slideMotor);
        slideCommand = new SlideCommand(slideSubsystem, () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        armSubsystem = new ArmSubsystem(gbServoRight, gbServoLeft);

        carouselSubsystem = new CarouselSubsystem(duckMotor);

        intakeLiftSubsystem = new IntakeLiftSubsystem(iLifterServo);

        // Instant commands
        intakeRunCommand = new InstantCommand(() -> {
            if (intakeMotor.get() != 0)
                intakeSubsystem.stopIntake();
            else intakeSubsystem.runIntake();
        }, intakeSubsystem);

        intakeReverseCommand = new InstantCommand(() -> {
            intakeSubsystem.reverseIntake();
        }, intakeSubsystem);

        liftIntakeCommand = new InstantCommand(() -> {
            if (intakeLiftSubsystem.isStraight || iLifterServo.getPosition() == 0.0)
                intakeLiftSubsystem.lifterIntakePos();
            else intakeLiftSubsystem.lifterStraightPos();
        }, intakeLiftSubsystem);

        carouselCommand = new InstantCommand(() -> {
            if (carouselSubsystem.isCarouselRunning)
                carouselSubsystem.stopCarousel();
            else carouselSubsystem.runCarousel();
        }, carouselSubsystem);

        useSlideCommand = new InstantCommand(() -> {
            // To make sure our cycles are spread out evenly,
            // we place the first minerals further back and then gradually move forward.
            // We use cycleCount to keep track of our scoring cycles.
            intakeLiftSubsystem.lifterIntakePos();
            depositSubsystem.closeDeposit();
            armSubsystem.levelIntermediateArm();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            intakeSubsystem.reverseIntake();
            turretSubsystem.moveTurretToHub();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            intakeLiftSubsystem.lifterStraightPos();
            intakeSubsystem.stopIntake();
            slideSubsystem.moveSlideToHubLevel3();
            armSubsystem.level3Arm();
        }, intakeSubsystem, depositSubsystem, armSubsystem, slideSubsystem, turretSubsystem, intakeLiftSubsystem);

        returnSlideCommand = new InstantCommand(() -> {
            // Open deposit
            depositSubsystem.openDeposit();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            armSubsystem.levelIntermediateArm();

            scoreTimer = new Timing.Timer(150);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            intakeLiftSubsystem.lifterIntakePos();
            slideSubsystem.moveSlideToIntermediate();

            scoreTimer = new Timing.Timer(150);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            // Move turret and slide back to home position
            turretSubsystem.moveTurretToHome();

            scoreTimer = new Timing.Timer(500);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            slideSubsystem.moveSlideToHome();

            scoreTimer = new Timing.Timer(250);
            scoreTimer.start();
            while (!scoreTimer.done())
            {
                // Wait for timer to end
            }
            scoreTimer.pause();

            armSubsystem.homeArm();
            intakeSubsystem.runIntake();
            depositSubsystem.middleDeposit();
            slideSubsystem.cycleCount++;
        }, slideSubsystem, armSubsystem, intakeSubsystem, turretSubsystem, depositSubsystem, intakeLiftSubsystem);

        moveTurretLeft = new InstantCommand(() -> {
           turretSubsystem.turretManualControl(-0.01);
        }, turretSubsystem);
        moveTurretRight = new InstantCommand(() -> {
            turretSubsystem.turretManualControl(0.01);
        }, turretSubsystem);

        // State the buttons that run commands
        // Using a PS4 Controller: Cross = A, Circle = B, Triangle = Y, Square = X
        Button intakeRunButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeRunCommand);
        Button intakeReverseButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(intakeReverseCommand);
        Button startCarouselButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(carouselCommand);
        Button liftIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(liftIntakeCommand);

        Button useSlideButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(useSlideCommand);
        Button returnSlideButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(returnSlideCommand);

        Button moveTurretLeftButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(moveTurretLeft);
        Button moveTurretRightButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(moveTurretRight);

        // Register subsystems and set their default commands (default commands = commands that run all the time)
        register(driveSubsystem, depositSubsystem, intakeSubsystem, carouselSubsystem, slideSubsystem, turretSubsystem, armSubsystem, intakeLiftSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        slideSubsystem.setDefaultCommand(slideCommand);
    }
}