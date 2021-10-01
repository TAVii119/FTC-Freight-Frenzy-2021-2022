package org.firstinspires.ftc.teamcode.test.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;



public class ArmSubsystem extends SubsystemBase {

    public final Servo RightarmRotation;
    public final Servo LeftarmRotation;
    int currentLevel = 0;
    double currentPosition = 0 ;

    public ArmSubsystem(Servo RArmRotation, Servo LArmRotation)
    {
        RightarmRotation= RArmRotation;
        LeftarmRotation = LArmRotation;
    }

    public void level1() {
        /*RightarmRotation.setPosition(0.5);
        LeftarmRotation.setPosition(0.5);*/
        currentPosition = 0.2;
        currentLevel = 1;
    }

    public void level2() {
        /*RightarmRotation.setPosition(0.5);
        LeftarmRotation.setPosition(0.5);*/
        currentPosition = 0.5;
        currentLevel = 2;
    }

    public void level3() {
        /*RightarmRotation.setPosition(0.5);
        LeftarmRotation.setPosition(0.5);*/
        currentPosition = 0.7;
        currentLevel = 3;
    }

    public void levelIntake() {
        currentPosition = 0.0;
        currentLevel = 0;
    }

    public int getCurrentLevel() {
        return currentLevel;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }
}
