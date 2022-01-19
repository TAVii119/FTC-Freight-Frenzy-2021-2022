package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    public Motor intakeMotor;
    private double intakePower;

    public IntakeSubsystem(Motor IntakeMotor){

        this.intakeMotor= IntakeMotor;
    }

    public void runIntake() {
        this.intakePower = 1;
    }

    public void stopIntake() {
        this.intakePower = 0;
    }

    public void reverseIntake() {
        this.intakePower = -1;
    }

    public double getIntakePower() {
        return this.intakePower;
    }
}
