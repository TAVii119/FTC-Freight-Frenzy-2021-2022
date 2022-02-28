package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Timing;

public class DepositSubsystem extends SubsystemBase {

    public final Servo deposit1;

    public double deposit1Open = 0;
    public double deposit1Close = 0.36;
    public double deposit1Ramp = 0.09;
    public double deposit1Intermediate = 0.12;

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
    isDepositClosed = false;}

    public void closeDeposit() {
        deposit1.setPosition(deposit1Close);
        isDepositClosed = true;
    }

}