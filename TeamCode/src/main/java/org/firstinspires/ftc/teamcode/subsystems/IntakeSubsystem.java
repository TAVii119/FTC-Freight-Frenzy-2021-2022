package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    public Motor intakeMotor;

    public IntakeSubsystem(Motor intakeMotor){
        this.intakeMotor = intakeMotor;
    }

    public void runIntake() {
        intakeMotor.set(0.8);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void reverseIntake() {
        intakeMotor.set(-1);
    }

}