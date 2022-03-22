package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TSESubsystem extends SubsystemBase {
    public Servo tseArmServo;
    public Servo tseClawServo;
    public Servo tseAngleServo;

    // Arm Servo
    private double pickUpPos = 0.84;
    private double waitPos = 0.18;
    private double prepareTSEPos = 0;
    private double scoreTSEPos = 0.52;
    private double initPos = 0;

    // Claw Servo
    private double gripPos = 0;
    private double releasePos = 0.42;

    // Angle Servo
    private double pickUpAnglePos = 0;
    private double scoreAnglePos = 0;

    public TSESubsystem(Servo tseArmServo, Servo tseClawServo, Servo tseAngleServo){

        this.tseArmServo = tseArmServo;
        this.tseClawServo = tseClawServo;
        this.tseAngleServo = tseAngleServo;
    }

    public void TSEManualControl(double position) {
        tseArmServo.setPosition(tseArmServo.getPosition() + position);
    }

    public void tsePickup(){
        tseArmServo.setPosition(pickUpPos);
    }

    public void tseWait(){
        tseArmServo.setPosition(waitPos); }

    public void initPos() {
    tseArmServo.setPosition(initPos);
    }

    public void tseScore(){
        tseArmServo.setPosition(scoreTSEPos);
    }

    public void tseClawRelease(){
        tseClawServo.setPosition(releasePos);
    }
    public void tseClawGrip(){
        tseClawServo.setPosition(gripPos);
    }

    public void tseAnglePickUp(){
        tseAngleServo.setPosition(pickUpAnglePos);
    }

    public void tseAngleScore(){
        tseAngleServo.setPosition(scoreAnglePos);
    }
}
