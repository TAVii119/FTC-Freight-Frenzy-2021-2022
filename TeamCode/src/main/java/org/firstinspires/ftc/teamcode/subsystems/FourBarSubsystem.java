package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo gbServoLeft, gbServoRight;
    private double fourBarTop = 0.64;
    private double fourBarMid = 0.76;
    private double fourBarLow = 0.86;
    private double fourBarIntake = 0.03;
    double fourBarPosition = 0.02;

    public FourBarSubsystem(Servo gbServoLeft, Servo gbServoRight){
        this.gbServoLeft = gbServoLeft;
        this.gbServoRight = gbServoRight;
    }

    public void fourBarTopPos(){
        fourBarPosition = fourBarTop;
    }

    public void fourBarMidPos(){
        fourBarPosition = fourBarMid;
    }

    public void fourBarLowPos(){
        fourBarPosition = fourBarLow;
    }


    public void fourBarIntakePos(){
        fourBarPosition = fourBarIntake;
    }

    public double getFourBarPosition(){ return fourBarPosition;
    }
}