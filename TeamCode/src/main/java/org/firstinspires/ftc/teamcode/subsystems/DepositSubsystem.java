package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositSubsystem extends SubsystemBase {

    public final Servo depositServo;
    public double depositOpen = 0;
    public double depositMiddle = 0.1;
    public double depositClose = 0.3;

    public DepositSubsystem(Servo depositServo) {
        this.depositServo = depositServo;
    }

    public void openDeposit() {
        depositServo.setPosition(depositOpen);
    }

    public void middleDeposit() { depositServo.setPosition(depositMiddle); }

    public void closeDeposit() {
        depositServo.setPosition(depositClose);
    }
}