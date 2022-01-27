/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Autonomous Vectron")
public class AutonomousVectron extends LinearOpMode {

    /* Declare OpMode members. */

    static final double     COUNTS_PER_MOTOR_REV    = 384.5;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     speed             = 0.6;
    static final double     turnSpeed              = 0.5;

    // Declare motors and servos
    private DcMotor l1 = null;
    private DcMotor l2 = null;
    private DcMotor l3 = null;
    private DcMotor r1 = null;
    private DcMotor r2 = null;
    private DcMotor r3 = null;
    private DcMotor intakeMotor = null;
    private DcMotor duckRightMotor = null;

    private Servo gbServoRight = null;
    private Servo gbServoLeft = null;
    private Servo depositServo = null;
    private Servo iLifterServo = null;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        l1 = hardwareMap.get(DcMotor.class, "l1");
        l2 = hardwareMap.get(DcMotor.class, "l2");
        l3 = hardwareMap.get(DcMotor.class, "l3");
        r1 = hardwareMap.get(DcMotor.class, "r1");
        r2 = hardwareMap.get(DcMotor.class, "r2");
        r3 = hardwareMap.get(DcMotor.class, "r3");
        duckRightMotor = hardwareMap.get(DcMotor.class, "duckRightMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        iLifterServo = hardwareMap.get(Servo.class, "iLifterServo");

        r1.setDirection(DcMotor.Direction.REVERSE);
        r2.setDirection(DcMotor.Direction.REVERSE);
        r3.setDirection(DcMotor.Direction.REVERSE);

        duckRightMotor.setDirection(DcMotor.Direction.REVERSE);
        gbServoLeft.setDirection(Servo.Direction.REVERSE);
        iLifterServo.setDirection(Servo.Direction.REVERSE);

        iLifterServo.setPosition(0);
        gbServoLeft.setPosition(0);
        gbServoRight.setPosition(0);
        depositServo.setPosition(0.49);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Drive to shipping hub
        encoderDrive(0.5, -10, -10);

        // Drop intake
        iLifterServo.setPosition(0.25);
        sleep(500);
        // Move fourbar to position
        gbServoLeft.setPosition(0.55);
        gbServoRight.setPosition(0.55);
        sleep(1500);
        // Push mineral
        depositServo.setPosition(0.74);
        sleep(200);
        // Open deposit
        depositServo.setPosition(0.5);
        sleep(200);
        // Push mineral
        depositServo.setPosition(0.74);
        sleep(200);
        // Open deposit
        depositServo.setPosition(0.5);
        sleep(200);
        // Push mineral
        depositServo.setPosition(0.74);
        sleep(200);
        // Open deposit
        depositServo.setPosition(0.5);
        sleep(200);
        // Return fourbar
        gbServoLeft.setPosition(0.05);
        gbServoRight.setPosition(0.05);
        sleep(1500);
        // Return intake
        iLifterServo.setPosition(0.1);

        // Rotate robot
        encoderDrive(0.2, -13, 13);
        sleep(200);

        // Park in warehouse
        encoderDrive(1, 50, 50);


        sleep(30000);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = l1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = r1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            l1.setTargetPosition(newLeftTarget);
            l2.setTargetPosition(newLeftTarget);
            l3.setTargetPosition(newLeftTarget);
            r1.setTargetPosition(newRightTarget);
            r2.setTargetPosition(newRightTarget);
            r3.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            l1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            l1.setPower(Math.abs(speed));
            l2.setPower(Math.abs(speed));
            l3.setPower(Math.abs(speed));
            r1.setPower(Math.abs(speed));
            r2.setPower(Math.abs(speed));
            r3.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop org.firstinspires.ftc.teamcode.test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop org.firstinspires.ftc.teamcode.test.
            while (opModeIsActive() && (l1.isBusy() && l2.isBusy() && l3.isBusy() && r1.isBusy() && r2.isBusy() && r3.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.update();
            }

            // Stop all motion;
            l1.setPower(0);
            l2.setPower(0);
            l3.setPower(0);
            r1.setPower(0);
            r2.setPower(0);
            r3.setPower(0);

            // Turn off RUN_TO_POSITION
            l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
