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

    public void moveTurretToHub() {
        // set the run mode
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        turretMotor.setPositionCoefficient(0.05);
        double kP = turretMotor.getPositionCoefficient();

        // set the target position
        turretMotor.setTargetPosition(turretHubPos); // an integer representing desired tick count

        turretMotor.set(0);

        // set the tolerance
        turretMotor.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!turretMotor.atTargetPosition()) {
            turretMotor.set(0.75);
        }
        turretMotor.stopMotor(); // stop the motor

        /* ALTERNATIVE TARGET DISTANCE */

        /*
        // configure a distance per pulse,
        // which is the distance traveled in a single tick
        // dpp = distance traveled in one rotation / CPR
        m_motor.setDistancePerPulse(0.015);

        // set the target
        m_motor.setTargetDistance(18.0);

        // this must be called in a control loop
        m_motor.set(0.5); // mode must be PositionControl
         */
    }
    public void moveTurretToHome() {
        // set the run mode
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        turretMotor.setPositionCoefficient(0.05);
        double kP = turretMotor.getPositionCoefficient();

        // set the target position
        turretMotor.setTargetPosition(turretHomePos); // an integer representing desired tick count

        turretMotor.set(0);

        // set the tolerance
        turretMotor.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!turretMotor.atTargetPosition()) {
            turretMotor.set(0.75);
        }
        turretMotor.stopMotor(); // stop the motor

        /* ALTERNATIVE TARGET DISTANCE */

        /*
        // configure a distance per pulse,
        // which is the distance traveled in a single tick
        // dpp = distance traveled in one rotation / CPR
        m_motor.setDistancePerPulse(0.015);

        // set the target
        m_motor.setTargetDistance(18.0);

        // this must be called in a control loop
        m_motor.set(0.5); // mode must be PositionControl
         */
    }

    public void moveTurretToInit() {
        // set the run mode
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        turretMotor.setPositionCoefficient(0.05);
        double kP = turretMotor.getPositionCoefficient();

        // set the target position
        turretMotor.setTargetPosition(turretInitPos); // an integer representing desired tick count

        turretMotor.set(0);

        // set the tolerance
        turretMotor.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!turretMotor.atTargetPosition()) {
            turretMotor.set(0.75);
        }
        turretMotor.stopMotor(); // stop the motor

        /* ALTERNATIVE TARGET DISTANCE */

        /*
        // configure a distance per pulse,
        // which is the distance traveled in a single tick
        // dpp = distance traveled in one rotation / CPR
        m_motor.setDistancePerPulse(0.015);

        // set the target
        m_motor.setTargetDistance(18.0);

        // this must be called in a control loop
        m_motor.set(0.5); // mode must be PositionControl
         */
    }


}