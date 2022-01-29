package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo armServo;
    private double armRetracted = 0.0;
    private double armLevel3 = 0.5;
    private double armLevel2 = 0.5;
    private double armLevel1 = 0.5;
    public boolean isArmExtended = false;

    public ArmSubsystem(Servo armServo) {
        this.armServo = armServo;
    }

    public void retractArm() {
        armServo.setPosition(armRetracted);
    }

    public void level3Arm() {
        armServo.setPosition(armLevel3);
    }
}