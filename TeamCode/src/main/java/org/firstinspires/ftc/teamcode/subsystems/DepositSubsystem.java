package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Timing;

public class DepositSubsystem extends SubsystemBase {

    public final Servo deposit1;

    public double deposit1Open = 0.16;
    public double deposit1Close = 0.43;
    public double deposit1Push = 0.64;

    public boolean isDepositClosed = false;
    Timing.Timer scoreTimer;

    public DepositSubsystem(Servo deposit1) {
        this.deposit1 = deposit1;
    }

    public void openDeposit() {
        deposit1.setPosition(deposit1Open);
        isDepositClosed = false;
    }

    public void pushDeposit() {
        deposit1.setPosition(deposit1Push);
    isDepositClosed = false;}

    public void closeDeposit() {
        deposit1.setPosition(deposit1Close);
        isDepositClosed = true;
    }

}