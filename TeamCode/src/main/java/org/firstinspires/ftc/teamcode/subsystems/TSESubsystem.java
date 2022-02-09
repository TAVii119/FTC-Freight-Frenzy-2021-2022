package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TSESubsystem extends SubsystemBase {
    public Servo tseServo;
    private double pickUpPos = 0.73;
    private double waitPos = 0.44;
    private double initPos = 0.04;

    public TSESubsystem(Servo tseServo){
        this.tseServo = tseServo;
    }

    public void TSEManualControl(double position) {
        tseServo.setPosition(tseServo.getPosition() + position);
    }

    public void TSEPickup(){
        tseServo.setPosition(pickUpPos);
    }
    public void TSEWait() { tseServo.setPosition(waitPos); }
    public void initPos(){
        tseServo.setPosition(initPos);
    }
}
