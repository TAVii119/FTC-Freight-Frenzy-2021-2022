package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo gbServoLeft, gbServoRight;
    private double fourBarTop = 0.58;
    private double fourBarMid = 0.70;
    private double fourBarLow = 0.83;
    private double fourBarIntake = 0.028;
    double fourBarPosition = 0.02;
    public boolean fourBarTopCheck = false;
    public boolean fourBarMidCheck = false;
    public boolean fourBarLowCheck = false;

    public FourBarSubsystem(Servo gbServoLeft, Servo gbServoRight){
        this.gbServoLeft = gbServoLeft;
        this.gbServoRight = gbServoRight;
    }

    public void fourBarTopPos() {
        gbServoLeft.setPosition(fourBarTop);
        gbServoRight.setPosition(fourBarTop);
        fourBarMidCheck = false;
        fourBarLowCheck = false;
        fourBarTopCheck = true;
    }

    public void fourBarMidPos() {
        gbServoLeft.setPosition(fourBarMid);
        gbServoRight.setPosition(fourBarMid);
        fourBarLowCheck = false;
        fourBarTopCheck = false;
        fourBarMidCheck = true;
    }

    public void fourBarLowPos() {
        gbServoLeft.setPosition(fourBarLow);
        gbServoRight.setPosition(fourBarLow);
        fourBarMidCheck = false;
        fourBarTopCheck = false;
        fourBarLowCheck = true;
    }


    public void fourBarIntakePos(){
        gbServoLeft.setPosition(fourBarIntake);
        gbServoRight.setPosition(fourBarIntake);
    }



}