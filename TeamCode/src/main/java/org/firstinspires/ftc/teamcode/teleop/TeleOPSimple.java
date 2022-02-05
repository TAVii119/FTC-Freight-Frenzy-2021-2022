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

    private Servo gbServoRight = null;
    private Servo gbServoLeft = null;
    private Servo depositServo = null;
    private Servo iLifterServo = null;

    double lfPower, rfPower, lbPower, rbPower;
    boolean isDepositOpen = true;
    boolean isIntakeRunning = false;
    boolean isArmExtended = false;

    // Servo positions
    double depositClose = 0.00, depositOpen = 0.00;

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

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");

        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);

        gbServoLeft.setPosition(0);
        gbServoRight.setPosition(0);
        depositServo.setPosition(0);
        iLifterServo.setPosition(0);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // Set modes
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            if(gamepad1.dpad_up){
                iLifterServo.setPosition(iLifterServo.getPosition() +0.02);
                sleep(500);
            }
            if(gamepad1.dpad_down){
                iLifterServo.setPosition(iLifterServo.getPosition() -0.02);
                sleep(500);
            }

            if(gamepad2.dpad_up){
                gbServoRight.setPosition(gbServoRight.getPosition() + 0.02);
                gbServoLeft.setPosition(gbServoLeft.getPosition() +0.02);
                sleep(500);

            }
            if(gamepad2.dpad_down){
                gbServoRight.setPosition(gbServoRight.getPosition() - 0.02);
                gbServoLeft.setPosition(gbServoLeft.getPosition() - 0.02);
                sleep(500);
            }

            if(gamepad2.dpad_left){
                depositServo.setPosition(depositServo.getPosition() -0.02);
                sleep(500);
            }
            if(gamepad2.dpad_right){
                depositServo.setPosition(depositServo.getPosition() +0.02);
                sleep(500);
            }



            // Telemetry
            telemetry.addData( "> iLifter position" , iLifterServo.getPosition());
            telemetry.addData( "> Deposit Position" , depositServo.getPosition());
            telemetry.addData( "> Arm Position" , gbServoLeft.getPosition());
            telemetry.update();
        }
    }
}