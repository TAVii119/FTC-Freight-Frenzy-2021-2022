package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo gbServoLeft, gbServoRight;
    private double fourBarTopPos = 0.72;
    private double fourBarMidPos = 0.74;
    private double fourBarLowPos = 0.78;
    private double fourBarIntakePos = 0.015;
    private double fourBarIntermediatePos = 0.30;
    private double fourBarIntermediateScorePos = 0.03;
    private double fourBarSharedMiddlePos = 0.70;
    private double fourBarTSEPreparePos = 0.93;
    private double fourBarTSERaisePos = 0.64;
    private double fourBarPosition = 0;
    public boolean fourBarTopCheck = false;
    public boolean fourBarMidCheck = false;
    public boolean fourBarLowCheck = false;
    public boolean isMoving = false;

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

    public void fourBarIntermediate(){
        gbServoLeft.setPosition(fourBarIntermediatePos);
        gbServoRight.setPosition(fourBarIntermediatePos);
    }

    public void fourBarIntermediateScore() {
        gbServoLeft.setPosition(fourBarIntermediateScorePos);
        gbServoRight.setPosition(fourBarIntermediateScorePos);
    }

    public void fourBarSharedMiddlePos() {
        gbServoLeft.setPosition(fourBarSharedMiddlePos);
        gbServoRight.setPosition(fourBarSharedMiddlePos);
    }

    public void fourBarTSEPrepare(){
        gbServoLeft.setPosition(fourBarTSEPreparePos);
        gbServoRight.setPosition(fourBarTSEPreparePos);
    }

    public void fourBarTSERaise(){
        gbServoLeft.setPosition(fourBarTSERaisePos);
        gbServoRight.setPosition(fourBarTSERaisePos);
    }
}