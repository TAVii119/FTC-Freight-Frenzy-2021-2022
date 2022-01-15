package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TurretSubsystem  extends SubsystemBase {
    public DcMotor turretMotor;
    private int turretPosition = 0; // 0-Intake, 0.4 TOP
    public boolean turretRunning = false;

    public TurretSubsystem(DcMotor turretMotor){
        this.turretMotor = turretMotor;
    }

    public void rotateToShippingHub(int position) {
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor.setTargetPosition(position);
        this.turretMotor.setPower(1);
        this.turretRunning = true;
    }

    public void returnToIntake() {
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor.setTargetPosition(this.turretPosition);
        this.turretMotor.setPower(1);
        this.turretRunning = true;
    }


    public void stopTurret() {
        this.turretMotor.setPower(0);
        this.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.turretRunning = false;
}

}
