package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TSESubsystem extends SubsystemBase {
    public Servo tseArmServo;
    public Servo tseClawServo;

    // Arm Servo
    private double pickUpPos = 0.75;
    private double waitPos = 0.27;
    private double scoreTSE = 0.44;
    private double intermediatePos = 0.36;

    private double initPos = 0;

    // Claw Servo
    private double gripPos = 0.1;
    private double releasePos = 0.48;


    public boolean score1 = true;

    public TSESubsystem(Servo tseArmServo, Servo tseClawServo){

        this.tseArmServo = tseArmServo;
        this.tseClawServo = tseClawServo;
    }

    public  void TSEClawManualControl(double position){
        tseClawServo.setPosition(tseClawServo.getPosition() + position);
    }

    public void TSEManualControl(double position) {
        tseArmServo.setPosition(tseArmServo.getPosition() + position);
    }

    public void tsePickup(){
        tseArmServo.setPosition(pickUpPos);
    }

    public void tseWait() { tseArmServo.setPosition(waitPos); }

    public void initPos(){
        tseArmServo.setPosition(initPos);
    }

    public void tseScore(){
        tseArmServo.setPosition(scoreTSE);
        score1 = false;
    }

    public void tseClawRelease(){
        tseClawServo.setPosition(releasePos);
    }
    public void tseClawGrip(){
        tseClawServo.setPosition(gripPos);
    }
}
