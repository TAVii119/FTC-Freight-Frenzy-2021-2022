package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOPSimple", group="Test")

public class TeleOPSimple extends LinearOpMode {

    // Declare OpMode members
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private DcMotor intakeMotor = null;
    private DcMotor slideMotor = null;
    private DcMotor turretMotor = null;

    private Servo armServo = null;
    private Servo depositServo = null;

    double lfPower, rfPower, lbPower, rbPower;
    boolean isDepositOpen = true;
    boolean isIntakeRunning = false;
    boolean isArmExtended = false;

    // Servo positions
    double depositClose = 0.00, depositOpen = 0.00;
    double armRetracted = 0.00, armExtended = 0.00;

    // Slide positions
    int slideIntakePos = 0;
    int slideLevel3Pos = 500;

    // Turret positions
    int turretHomePos = 0;
    int turretHubPos = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        armServo = hardwareMap.get(Servo.class, "armServo");
        depositServo = hardwareMap.get(Servo.class, "depositServo");

        armServo.setPosition(0);
        depositServo.setPosition(0);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // Set modes
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Send calculated power to wheels
            lfPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            rfPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            lbPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rbPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

            lf.setPower(lfPower);
            rf.setPower(rfPower);
            lb.setPower(lbPower);
            rb.setPower(rbPower);

            // Intake control
            if (gamepad1.a) {
                if (isIntakeRunning) {
                    intakeMotor.setPower(0);
                    isIntakeRunning = false;
                } else {
                    intakeMotor.setPower(1);
                    isIntakeRunning = true;
                }
                sleep(50);
            }

            if (gamepad1.b) {
                if (isIntakeRunning) {
                    intakeMotor.setPower(0);
                    isIntakeRunning = false;
                } else {
                    intakeMotor.setPower(-1);
                    isIntakeRunning = true;
                }
                sleep(50);
            }

            // Extend scoring system
            if (gamepad1.right_bumper) {

                // Outtake freight
                intakeMotor.setPower(-1);
                isIntakeRunning = true;

                // Close deposit
                depositServo.setPosition(depositClose);
                isDepositOpen = false;

                // Move deposit arm and wait for it to fully extend
                armServo.setPosition(armExtended);
                isArmExtended = true;
                sleep(250);

                // Move turret to shipping hub position
                turretMotor.setTargetPosition(turretHubPos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(1);

                while (opModeIsActive() && turretMotor.isBusy()) {
                    telemetry.addData(">", "Turret rotating to hub position");
                    telemetry.update();
                }

                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Move slides to level 3 position
                slideMotor.setTargetPosition(slideLevel3Pos);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);

                while (opModeIsActive() && slideMotor.isBusy()) {
                    telemetry.addData(">", "Slides going to level 3");
                    telemetry.update();
                }

                slideMotor.setPower(0);
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Score freight and retract scoring system
            if (gamepad1.left_bumper) {

                // Intake freight
                intakeMotor.setPower(1);
                isIntakeRunning = true;

                // Open deposit
                depositServo.setPosition(depositOpen);
                isDepositOpen = true;

                // Wait a bit for freight to drop out of the deposit
                sleep(250);

                // Retract deposit arm and wait for it to fully retract
                armServo.setPosition(armRetracted);
                isArmExtended = false;
                sleep(250);

                // Move turret to home position
                turretMotor.setTargetPosition(turretHomePos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(1);

                while (opModeIsActive() && turretMotor.isBusy()) {
                    telemetry.addData(">", "Turret rotating to home position");
                    telemetry.update();
                }

                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Move slides to intake position
                slideMotor.setTargetPosition(slideIntakePos);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);

                while (opModeIsActive() && slideMotor.isBusy()) {
                    telemetry.addData(">", "Slides going to level 3");
                    telemetry.update();
                }

                slideMotor.setPower(0);
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Control deposit servo
            if (gamepad1.y) {
                if (isDepositOpen) {
                    depositServo.setPosition(depositClose);
                    isDepositOpen = false;
                } else {
                    depositServo.setPosition(depositOpen);
                    isDepositOpen = true;
                }
                sleep(50);
            }

            // Control arm servo
            if (gamepad1.x) {
                if (isArmExtended) {
                    armServo.setPosition(armRetracted);
                    isArmExtended = false;
                } else {
                    armServo.setPosition(armExtended);
                    isArmExtended = true;
                }
                sleep(50);
            }

            // Move slides manually
            slideMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // Telemetry
            telemetry.addData("> Slide position: ", slideMotor.getCurrentPosition());
            telemetry.addData("> Turret position: ", turretMotor.getCurrentPosition());
            telemetry.addData("> left stick y", -gamepad1.left_stick_y);
            telemetry.addData("> left stick x", gamepad1.left_stick_x);
            telemetry.addData("> right stick x", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
