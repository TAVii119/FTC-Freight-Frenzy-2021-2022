package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FourWheelDrive")

public class FourWheelDrive extends LinearOpMode {
    private DcMotor r1 = null;
    private DcMotor r2 = null;
    private DcMotor l1 = null;
    private DcMotor l2 = null;
    private Servo gbServoRight = null;
    private Servo gbServoLeft = null;
    double leftPower;
    double rightPower;
    
    @Override
    public void runOpMode() {
        l1 = hardwareMap.get(DcMotor.class, "l1");
        l2 = hardwareMap.get(DcMotor.class, "l2");
        r1 = hardwareMap.get(DcMotor.class, "r1");
        r2 = hardwareMap.get(DcMotor.class, "r2");

        l1.setDirection(DcMotor.Direction.REVERSE);
        l2.setDirection(DcMotor.Direction.REVERSE);

        gbServoLeft = hardwareMap.get(Servo.class, "gbServoLeft");
        gbServoRight = hardwareMap.get(Servo.class, "gbServoRight");

        gbServoLeft.setDirection(Servo.Direction.REVERSE);

        gbServoLeft.setPosition(0);
        gbServoRight.setPosition(0);
        
        
        
        


        WaitForStart();
        while (opModeIsActive()){
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            r1.setPower(rightPower);
            r2.setPower(rightPower);
            l2.setPower(leftPower);
            l1.setPower(leftPower);
            
            if(gamepad1.triangle){
               gbServoLeft.setPosition(0.50);
               gbServoRight.setPosition(0.50);
                
            }

        }
        
        
    
    }

    private void WaitForStart() {
    }


}
