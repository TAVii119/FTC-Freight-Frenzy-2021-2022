package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositSubsystem extends SubsystemBase {

    public final Servo depositServo;
    public int depositLevel = 0; // 0-Open, 1-Close, 2-PUSH.
    public double depositPosition = 0; // 0 - Open, 0.38 - Closed, 0.58 - Push
    public boolean depositOpen = true;

    public DepositSubsystem(Servo depositServo) {
        this.depositServo = depositServo;
        this.depositServo.setDirection(Servo.Direction.REVERSE);
    }

    public void openDeposit() {
        depositPosition = 0.27;
        depositLevel = 0;
        depositOpen = true;
    }

    public void closeDeposit() {
        depositPosition = 0.49;
        depositLevel = 1;
        depositOpen = false;
    }

    public void initServo() {
        depositServo.setPosition(0);
    }

    public double getDepositPosition(){
        return depositPosition;
    }
    public int getDepositLevel(){
        return depositLevel;
    }
}