package org.firstinspires.ftc.teamcode.subsystems;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

    /*
    This is our intake subsystem. Using one DC Motor we can intake our outtake (spit out) freight (game elements).
     */

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;
    FourBarSubsystem fourBarSubsystem;

    public IntakeSubsystem(Motor intakeMotor, FourBarSubsystem fourBarSubsystem) {
        this.intakeMotor = intakeMotor;
        this.fourBarSubsystem = fourBarSubsystem;
    }

    public void runIntake(double power) {
        intakeMotor.set(power);
    }
}