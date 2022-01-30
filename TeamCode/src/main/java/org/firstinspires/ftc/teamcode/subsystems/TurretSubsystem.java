package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
    public Servo turretServo;
    public double turretInitPos = 0.28;
    public double turretHomePos = 0.178;
    public double turretHubPos = 0.36;

    public TurretSubsystem(Servo turretServo) {
        this.turretServo = turretServo;
    }

    public void moveTurretToInit(){
        turretServo.setPosition(turretInitPos);
    }

    public void moveTurretToHome(){
        turretServo.setPosition(turretHomePos);
    }

    public void moveTurretToHub(){
        turretServo.setPosition(turretHubPos);
    }

}