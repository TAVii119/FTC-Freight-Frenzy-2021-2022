package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private Servo iLifterServo = null;

    double lfPower, rfPower, lbPower, rbPower;
    boolean isDepositOpen = true;
    boolean isIntakeRunning = false;
    boolean isArmExtended = false;

    // Servo positions
    double depositClose = 0.00, depositOpen = 0.00;
    double armRetracted = 0.00, armExtended = 0.00;

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
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");

        depositServo.setDirection(Servo.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.REVERSE);

        armServo.setPosition(0);
        depositServo.setPosition(0);
        iLifterServo.setPosition(0);

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

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                sleep(200);
            }

            // Outtake control
            if (gamepad1.b) {
                if (isIntakeRunning) {
                    intakeMotor.setPower(0);
                    isIntakeRunning = false;
                } else {
                    intakeMotor.setPower(-1);
                    isIntakeRunning = true;
                }
                sleep(200);
            }

            // Move slides manually
            //slideMotor.setPower(-gamepad2.left_stick_y);
            //turretMotor.setPower(-gamepad2.right_stick_y);

           // Manually control servos
            iLifterServo.setPosition(-gamepad2.right_stick_y);
            armServo.setPosition(-gamepad2.left_stick_x);
            depositServo.setPosition(-gamepad2.right_stick_x);

            // Telemetry
            telemetry.addData("> Slide position: ", slideMotor.getCurrentPosition());
            telemetry.addData("> Turret position: ", turretMotor.getCurrentPosition());
            telemetry.addData( "> iLifter position" , iLifterServo.getPosition());
            telemetry.addData( "> Deposit Position" , depositServo.getPosition());
            telemetry.addData( "> Arm Position" , armServo.getPosition());
            telemetry.update();
        }
    }
}