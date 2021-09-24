package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

    /*
    Tihs is our intake subsystem. Using one DC Motor we can intake our outtake (spit out) freight (game elements).
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