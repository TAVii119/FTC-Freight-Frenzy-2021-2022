package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TurretSubsystem extends SubsystemBase {
    public DcMotor turretMotor;
    private int turretInitPos = 75;
    private int turretHomePos = 0;
    private int turretHubPos = 100;
    public int turretManualPos = 0;

    public TurretSubsystem(DcMotor turretMotor) {
        this.turretMotor = turretMotor;
    }
    public void motorInit() {
        this.turretMotor.setTargetPosition(turretInitPos); // change for shipping hub position
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor.setPower(0.5);
    }
    public void moveToShippingHub() {
        this.turretMotor.setTargetPosition(turretHubPos); // change for shipping hub position
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor.setPower(1);
    }
    public void moveTurretBack() {
        this.turretMotor.setTargetPosition(turretHomePos); // change for robot front position
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor.setPower(1);
    }
}