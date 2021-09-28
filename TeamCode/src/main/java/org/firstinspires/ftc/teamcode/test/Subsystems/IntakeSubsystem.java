package org.firstinspires.ftc.teamcode.test.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

    /*
    Tihs is our intake subsystem. Using one DC Motor we can intake our outtake (spit out) freight (game elements).
     */

public class IntakeSubsystem extends SubsystemBase {

    private final Motor lintakeMotor;
    private final Motor rintakemotor;

    public IntakeSubsystem(Motor lIntakeMotor, Motor rIntakeMotor) {
        lintakeMotor = lIntakeMotor;
        rintakemotor=rIntakeMotor;
    }

    public void runIntake(double power) {
        lintakeMotor.set(power);
        rintakemotor.set(power);
    }

    public void stopIntake() {
        lintakeMotor.set(0);
        rintakemotor.set(0);
    }
}