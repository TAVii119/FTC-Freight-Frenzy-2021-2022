package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TurretSubsystem extends SubsystemBase {
    public Motor turretMotor;
    private int turretInitPos = 75;
    private int turretHomePos = 0;
    private int turretHubPos = 100;
    public int turretManualPos = 0;

    public TurretSubsystem(Motor turretMotor) {
        this.turretMotor = turretMotor;
    }

}