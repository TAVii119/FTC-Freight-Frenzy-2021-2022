package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLiftSubsystem extends SubsystemBase {
    public Servo iLifterServo;
    private double initPos = 0;
    private double intakePos = 0.27;
    private double fourBarPos = 0;
    public boolean isStraight = false;

    public IntakeLiftSubsystem(Servo iLifterServo) {
        this.iLifterServo = iLifterServo;
    }

    public void lifterInitPos() {
        iLifterServo.setPosition(initPos);
    }

    public void iLifterfourBarPos(){
        iLifterServo.setPosition(fourBarPos);
        isStraight = true;
    }

    public void lifterIntakePos() {
        iLifterServo.setPosition(intakePos);
        isStraight = false;
    }
}