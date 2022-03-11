package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TSESubsystem extends SubsystemBase {
    public Servo tseServo;
    private double pickUpPos = 0.825;
    private double waitPos = 0.34;
    private double prepareTSE = 0.57;
    private double scoreTSE = 0.57;
    private double initPos = 0;

    public TSESubsystem(Servo tseServo){
        this.tseServo = tseServo;
    }

    public void TSEManualControl(double position) {
        tseServo.setPosition(tseServo.getPosition() + position);
    }

    public void tsePrepare(){
        tseServo.setPosition(prepareTSE);
    }
    public void tsePickup(){
        tseServo.setPosition(pickUpPos);
    }
    public void tseWait() { tseServo.setPosition(waitPos); }
    public void tseScore() { tseServo.setPosition(scoreTSE); }
    public void initPos(){
        tseServo.setPosition(initPos);
    }
}
