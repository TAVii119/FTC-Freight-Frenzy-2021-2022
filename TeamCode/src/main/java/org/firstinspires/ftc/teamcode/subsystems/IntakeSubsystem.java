package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
This class was created by Botosan Octavian on September 24, 2021.
This is a subsystem for the freight intake that we use.
 */

public class IntakeSubsystem extends SubsystemBase {

    private Motor intakeMotor;

    public IntakeSubsystem(Motor IntakeMotor) {
        intakeMotor = IntakeMotor;
    }

    public void runIntake(double power) {
        intakeMotor.set(power);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
}