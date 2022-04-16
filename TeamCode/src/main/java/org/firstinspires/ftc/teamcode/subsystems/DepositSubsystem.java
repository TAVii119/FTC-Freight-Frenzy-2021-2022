package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Timing;

public class DepositSubsystem extends SubsystemBase {

    public final Servo deposit1;

    public double deposit1Open = 0.02;
    public double deposit1Close = 0.34;
    public double deposit1Ramp = 0.09;
    public double deposit1Intermediate = 0.12;
    public double depositTSEClaw = 0.57;
    public double depositTSERelease = 0.22;
    public double depositPrepare = 0.25;

    public boolean isDepositClosed = false;
    Timing.Timer scoreTimer;

    public DepositSubsystem(Servo deposit1) {
        this.deposit1 = deposit1;
    }

    public void openDeposit() {
        deposit1.setPosition(deposit1Open);
        isDepositClosed = false;
    }

    public void depositIntermediate() {
        deposit1.setPosition(deposit1Intermediate);
        isDepositClosed = false;
    }

    public void closeDeposit() {
        deposit1.setPosition(deposit1Close);
        isDepositClosed = true;
    }

    public void depositTSEClaw() {
        deposit1.setPosition(depositTSEClaw);
        isDepositClosed = true;
    }

    public void depositTSERelease() {
        deposit1.setPosition(deposit1Open);
        isDepositClosed = true;
    }

    public void depositPrepare() {
        deposit1.setPosition(depositPrepare);
        isDepositClosed = true;
    }
}