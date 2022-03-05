package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Odometry Tester", group="Test")
public class OdometryTester extends LinearOpMode {

    DcMotor leftEncoder, rightEncoder, strafeEncoder;

    @Override
    public void runOpMode() {

        leftEncoder = hardwareMap.get(DcMotor.class, "lf");
        rightEncoder = hardwareMap.get(DcMotor.class, "rf");
        strafeEncoder = hardwareMap.get(DcMotor.class, "lb");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
            telemetry.addData("strafeEncoder", strafeEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
