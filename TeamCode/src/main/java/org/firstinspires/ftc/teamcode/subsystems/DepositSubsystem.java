package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositSubsystem extends SubsystemBase {

    public final Servo depositServo;
    public double depositOpen = 0;
    public double depositPush = 0.58;
    public double depositClose = 0.38;
    public boolean isDepositClosed = false;

    public DepositSubsystem(Servo depositServo) {
        this.depositServo = depositServo;
    }

    public void openDeposit() {
        depositServo.setPosition(depositOpen);
        isDepositClosed = false;
    }

    public void pushDeposit() { depositServo.setPosition(depositPush);
    isDepositClosed = false;}

    public void closeDeposit() {
        depositServo.setPosition(depositClose);
        isDepositClosed = true;
    }
}