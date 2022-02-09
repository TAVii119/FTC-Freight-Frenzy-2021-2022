package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo gbServoLeft, gbServoRight;
    private double fourBarTop = 0.57;
    private double fourBarMid = 0.72;
    private double fourBarLow = 0.83;
    private double fourBarIntake = 0.03;
    double fourBarPosition = 0.02;

    public FourBarSubsystem(Servo gbServoLeft, Servo gbServoRight){
        this.gbServoLeft = gbServoLeft;
        this.gbServoRight = gbServoRight;
    }

    public void fourBarTopPos() {
        gbServoLeft.setPosition(fourBarTop);
        gbServoRight.setPosition(fourBarTop);
    }

    public void fourBarMidPos() {
        gbServoLeft.setPosition(fourBarMid);
        gbServoRight.setPosition(fourBarMid);
    }

    public void fourBarLowPos() {
        gbServoLeft.setPosition(fourBarLow);
        gbServoRight.setPosition(fourBarLow);
    }


    public void fourBarIntakePos(){
        gbServoLeft.setPosition(fourBarIntake);
        gbServoRight.setPosition(fourBarIntake);
    }


}