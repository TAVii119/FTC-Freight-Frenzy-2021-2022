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
    private boolean intaking = false;
    private boolean running = false;

    public IntakeSubsystem(Motor IntakeMotor) {
        intakeMotor = IntakeMotor;
    }

    public void intake() {
        intakeMotor.set(1);
        intaking = true;
        running = true;
    }

    public void outtake() {
        intakeMotor.set(-1);
        intaking = false;
        running = true;
    }

    public void stop() {
        intakeMotor.stopMotor();
        intaking = false;
        running = false;
    }

    public boolean getIntaking() {
        return intaking;
    }

    public boolean getRunning() {
        return running;
    }
}