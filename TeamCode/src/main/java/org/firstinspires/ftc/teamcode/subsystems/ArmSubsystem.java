package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo gbServoRight;
    private Servo gbServoLeft;
    private double armHome = 0.03;
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

    public void moveArm(double position) {
        gbServoRight.setPosition(position);
        gbServoLeft.setPosition(position);
        isArmExtended = true;
    }

    public void levelIntermediateArm(){
        gbServoRight.setPosition(intermediatePosition);
        gbServoLeft.setPosition(intermediatePosition);
        isArmExtended = true;
    }
}