package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo leftServo, rightServo;
    public double[] Levels = new double[]{0.02, 0.06, 0.58, 0.70, 0.79}; // INTAKE, WAIT, TOP, MID, LOW
    public double fourBarPosition = 0.02;
    int fourBarLevel = 0;
    public int desiredLevel = 0;

    public FourBarSubsystem(Servo left, Servo right){
        leftServo = left;
        rightServo = right;
        leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setLevel(){
        fourBarPosition = Levels[desiredLevel];
        fourBarLevel = desiredLevel;
    }

    public void setLevelWait(){
        fourBarPosition = Levels[1];
        fourBarLevel = 1;
    }

    public void setLevelIntake(){
        fourBarPosition = Levels[0];
        fourBarLevel = 0;
    }

    public void setDesiredLevel(int level){
        desiredLevel = level;
    }

    public int getFourBarLevel() { return fourBarLevel; }

    public double getFourBarPosition() { return fourBarPosition; }
}
