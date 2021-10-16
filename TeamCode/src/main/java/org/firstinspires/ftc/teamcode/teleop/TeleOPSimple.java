package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOPSimple", group="Test")

public class TeleOPSimple extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor l1 = null; //l1, l2, l3 ; r1, r2, r3
    private DcMotor l2 = null;
    private DcMotor l3 = null;
    private DcMotor r1 = null;
    private DcMotor r2 = null; //l1, l2, l3 ; r1, r2, r3
    private DcMotor r3 = null;
    private DcMotor intakeMotor = null;
    private Servo gbServoRight = null;
    private Servo gbServoLeft = null;
    private Servo depositServo = null;
    double leftPower;
    double rightPower;
    boolean openDeposit = true;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        l1 = hardwareMap.get(DcMotor.class, "l1");
        l2 = hardwareMap.get(DcMotor.class, "l2");
        l3 = hardwareMap.get(DcMotor.class, "l3");
        r1 = hardwareMap.get(DcMotor.class, "r1");
        r2 = hardwareMap.get(DcMotor.class, "r2");
        r3 = hardwareMap.get(DcMotor.class, "r3");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        depositServo = hardwareMap.get(Servo.class, "depositServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        r1.setDirection(DcMotor.Direction.REVERSE);
        r2.setDirection(DcMotor.Direction.REVERSE);
        r3.setDirection(DcMotor.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        depositServo.setDirection(Servo.Direction.REVERSE);
        gbServoLeft.setPosition(0);
        gbServoRight.setPosition(0);
        depositServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Send calculated power to wheels

            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            l1.setPower(leftPower);
            l2.setPower(leftPower);
            l3.setPower(leftPower);
            r1.setPower(rightPower);
            r2.setPower(rightPower);
            r3.setPower(rightPower);

            // Top Level
            if (gamepad2.triangle) {
                gbServoLeft.setPosition(0.58);
                gbServoRight.setPosition(0.58);
                depositServo.setPosition(0.38);
                openDeposit = false;
                sleep(200);
            }

            // MID LEVEL
            if (gamepad2.circle) {
                gbServoLeft.setPosition(0.70);
                gbServoRight.setPosition(0.70);
                depositServo.setPosition(0.38);
                openDeposit = false;
                sleep(200);
            }

            // LOW LEVEL
            if (gamepad2.cross) {
                gbServoLeft.setPosition(0.79);
                gbServoRight.setPosition(0.79);
                depositServo.setPosition(0.38);
                openDeposit = false;
                sleep(200);
            }

            // Deposit
            if (gamepad2.square && openDeposit) {
                depositServo.setPosition(0.38);
                openDeposit = false;
                sleep(200);
            } else if (gamepad2.square && !openDeposit) {
                depositServo.setPosition(0);
                openDeposit = true;
                sleep(200);
            }
            // PUSH MINERALS
            if(gamepad1.cross){
                depositServo.setPosition(0.59);
                openDeposit = false;
            }

            // DE ASTA NU MERGEA CODUL, CA ASTA II DADEA POZITIA 0 IN CONTINUU
           // depositServo.setPosition(-gamepad2.right_stick_y);
          //  gbServoLeft.setPosition(-gamepad2.left_stick_y);
          //  gbServoRight.setPosition(-gamepad2.left_stick_y);

            // Control Intake
            intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            if (gamepad1.right_trigger > 0.1) { // INTAKE POS
                gbServoLeft.setPosition(0.02);
                gbServoRight.setPosition(0.02);
                depositServo.setPosition(0);
                openDeposit = true;
            }

            if(gamepad2.right_trigger > 0.1) // WAIT POS
            { gbServoLeft.setPosition(0.06);
            gbServoRight.setPosition(0.06);
            depositServo.setPosition(0.38);
            openDeposit = false;
            }

            // Telemetry
            telemetry.addData("Gearbox Pos:", -gamepad2.left_stick_y);
            telemetry.addData("gbServoLeft Pos:", gbServoLeft.getPosition());
            telemetry.addData("gbServoRight Pos:", gbServoRight.getPosition());
            telemetry.addData("Deposit Pos:", -gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}
