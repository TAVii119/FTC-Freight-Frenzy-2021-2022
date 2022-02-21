package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo gbServoLeft, gbServoRight;
    private double fourBarTopPos = 0;
    private double fourBarMidPos = 0;
    private double fourBarLowPos = 0;
    private double fourBarIntakePos = 0;
    double fourBarPosition = 0.02;
    public boolean fourBarTopCheck = false;
    public boolean fourBarMidCheck = false;
    public boolean fourBarLowCheck = false;

    public FourBarSubsystem(Servo gbServoLeft, Servo gbServoRight){
        this.gbServoLeft = gbServoLeft;
        this.gbServoRight = gbServoRight;
    }

    public void fourBarTop() {
        gbServoLeft.setPosition(fourBarTopPos);
        gbServoRight.setPosition(fourBarTopPos);
        fourBarMidCheck = false;
        fourBarLowCheck = false;
        fourBarTopCheck = true;
    }

    public void fourBarMid() {
        gbServoLeft.setPosition(fourBarMidPos);
        gbServoRight.setPosition(fourBarMidPos);
        fourBarLowCheck = false;
        fourBarTopCheck = false;
        fourBarMidCheck = true;
    }

    public void fourBarLow() {
        gbServoLeft.setPosition(fourBarLowPos);
        gbServoRight.setPosition(fourBarLowPos);
        fourBarMidCheck = false;
        fourBarTopCheck = false;
        fourBarLowCheck = true;
    }


    public void fourBarIntake(){
        gbServoLeft.setPosition(fourBarIntakePos);
        gbServoRight.setPosition(fourBarIntakePos);
    }



}