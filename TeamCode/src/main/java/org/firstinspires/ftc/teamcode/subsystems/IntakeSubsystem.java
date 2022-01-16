package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;

    public IntakeSubsystem(Motor IntakeMotor){
        intakeMotor = IntakeMotor;
    }

    public void runIntake(double power){
        intakeMotor.set(power);
    }
}
