package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo armServo;
    private double armRetracted = 0.0;
    private double armExtended = 0.5  ;
    public boolean isArmExtended = false;

    public ArmSubsystem(Servo armServo) {
        this.armServo = armServo;
    }

    public void extendArm() {
        this.armServo.setPosition(armExtended);
        isArmExtended = true;
    }

    public void retractArm() {
        this.armServo.setPosition(armRetracted);
        isArmExtended = false;
    }

    public void initServo() {
        this.armServo.setPosition(0);
    }
}
