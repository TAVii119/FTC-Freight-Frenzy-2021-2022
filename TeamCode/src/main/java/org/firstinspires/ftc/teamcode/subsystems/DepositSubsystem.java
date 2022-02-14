package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Timing;

public class DepositSubsystem extends SubsystemBase {

    public final Servo deposit1;
    public final Servo deposit2;

    public double deposit1Open = 0.16;
    public double deposit1Close = 0.43;
    public double deposit1Push = 0.64;

    public double deposit2Open = 0.28;
    public double deposit2Close = 0;

    public boolean isDepositClosed = false;
    Timing.Timer scoreTimer;

    public DepositSubsystem(Servo deposit1, Servo deposit2) {
        this.deposit1 = deposit1;
        this.deposit2 = deposit2;
    }

    public void openDeposit() {
        deposit2.setPosition(deposit2Close);
        deposit1.setPosition(deposit1Open);
        isDepositClosed = false;
    }

    public void pushDeposit() {
        deposit2.setPosition(deposit2Open);
        deposit1.setPosition(deposit1Push);
    isDepositClosed = false;}

    public void closeDeposit() {
        deposit2.setPosition(deposit2Close);
        deposit1.setPosition(deposit1Close);
        isDepositClosed = true;
    }

}