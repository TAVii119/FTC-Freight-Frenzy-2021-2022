package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositSubsystem extends SubsystemBase {
    public final Servo depositArmRotation;
    public int depositLevel = 0; // 0-Open, 1-Close, 2-PUSH.
    public double depositPosition = 0; // 0 - Open, 0.38 - Closed, 0.58 - Push
    public DepositSubsystem(Servo DArmRotation) {
        depositArmRotation=DArmRotation;
        depositArmRotation.setDirection(Servo.Direction.REVERSE);
    }
    public void openDeposit() {
        depositPosition = 0;
        depositLevel=0;
    }
    public void closeDeposit() {
        depositPosition = 0.38;
        depositLevel=1;
    }
    public void pushDeposit() {
        depositPosition = 0.58;
        depositLevel=2;
    }

    public double getDepositPosition(){
        return depositPosition;
    }
    public int getDepositLevel(){
        return depositLevel;
    }
}
