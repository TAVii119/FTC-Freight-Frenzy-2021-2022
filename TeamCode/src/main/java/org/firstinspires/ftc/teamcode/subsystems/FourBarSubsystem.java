package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSubsystem extends SubsystemBase {
    public Servo barServo;
    public double[] Levels = new double[]{0.02, 0.05, 0.58}; // INTAKE, WAIT, TOP;
    public double BarPosition = 0.02;
    int BarLevel = 0;

    public FourBarSubsystem(Servo barServo){
        this.barServo = barServo;
        barServo.setDirection(Servo.Direction.REVERSE);
    }

    public void moveBarTop(){
        BarPosition = Levels[2];
        BarLevel = 2;
    }

    public void setLevelWait(){
        BarPosition = Levels[1];
        BarLevel = 1;
    }

    public void setLevelIntake(){
        BarPosition = Levels[0];
        BarLevel = 0;
    }

    public int getBarLevel() { return BarLevel; }

    public double getBarPosition() { return BarPosition; }
}
