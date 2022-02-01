package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private double armHome = 0.03;
    private double armLevel3 = 0.60;
    private double armLevel2 = 0.29;
    private double armLevel1 = 0.29;
    private double intermediatePosition = 0.36;

    public boolean isArmExtended = false;

    public ArmSubsystem(Servo gbServoRight, Servo gbServoLeft) {
        this.gbServoRight = gbServoRight;
        this.gbServoLeft = gbServoLeft;
    }

    public void homeArm() {
        gbServoRight.setPosition(armHome);
        gbServoLeft.setPosition(armHome);
        isArmExtended = false;
    }

    public void level3Arm() {
        gbServoRight.setPosition(armLevel3);
        gbServoLeft.setPosition(armLevel3);
        isArmExtended = true;
    }

    public void level2Arm() {
        gbServoRight.setPosition(armLevel2);
        gbServoLeft.setPosition(armLevel2);
        isArmExtended = true;
    }

    public void level1Arm(){
        gbServoRight.setPosition(armLevel1);
        gbServoLeft.setPosition(armLevel1);
        isArmExtended = true;
    }

    public void levelIntermediateArm(){
        gbServoRight.setPosition(intermediatePosition);
        gbServoLeft.setPosition(intermediatePosition);
        isArmExtended = true;
    }
}