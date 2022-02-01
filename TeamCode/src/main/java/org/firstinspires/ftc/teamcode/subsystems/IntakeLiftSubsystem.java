package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLiftSubsystem extends SubsystemBase {
    private Servo iLifterServo;
    private double initPos = 0;
    private double straightPos = 0.04;
    private double intakePos = 0.33;
    public boolean isStraight = false;

    public IntakeLiftSubsystem(Servo iLifterServo) {
        this.iLifterServo = iLifterServo;
    }

    public void lifterInitPos() {
        iLifterServo.setPosition(initPos);
    }

    public void lifterStraightPos() {

        iLifterServo.setPosition(straightPos);
        isStraight = true;
    }

    public void lifterIntakePos() {

        iLifterServo.setPosition(intakePos);
        isStraight = false;
    }
}