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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="6WD Chassis", group="Test")

public class ChassisConcept extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor l1 = null; //l1, l2, l3 ; r1, r2, r3
    private DcMotor l2 = null;
    private DcMotor l3 = null;
    private DcMotor r1 = null;
    private DcMotor r2 = null; //l1, l2, l3 ; r1, r2, r3
    private DcMotor r3 = null;
    double leftPower;
    double rightPower;

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


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        r1.setDirection(DcMotor.Direction.REVERSE);
        r2.setDirection(DcMotor.Direction.REVERSE);
        r3.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            // Send calculated power to wheels

            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
            rightPower   = Range.clip(drive - turn, -1.0, 1.0);

            l1.setPower(leftPower);
            l2.setPower(leftPower);
            l3.setPower(leftPower);
            r1.setPower(rightPower);
            r2.setPower(rightPower);
            r3.setPower(rightPower);

            // Telemetry
        }
    }
}
