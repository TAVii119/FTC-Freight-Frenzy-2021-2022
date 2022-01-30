package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo armServo;
    private double armHome = 0.015;
    private double armLevel3 = 0.7;
    private double armLevel2 = 0.5;
    private double armLevel1 = 0.5;
    private double intermediatePosition = 0.28;

    public boolean isArmExtended = false;

    public ArmSubsystem(Servo armServo) {
        this.armServo = armServo;
    }

    public void homeArm() {
        armServo.setPosition(armHome);
        isArmExtended = false;
    }

    public void level3Arm() {
        armServo.setPosition(armLevel3);
        isArmExtended = true;
    }

    public void level2Arm() {
        armServo.setPosition(armLevel2);
        isArmExtended = true;
    }

    public void level1Arm(){
        armServo.setPosition(armLevel1);
        isArmExtended = true;
    }

    public void levelIntermediateArm(){
        armServo.setPosition(intermediatePosition);
        isArmExtended = true;
    }
}