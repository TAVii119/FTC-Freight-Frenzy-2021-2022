package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SlideSubsystem extends SubsystemBase {
    public Motor slideMotor;
    private int homePosition = 0;
    private int hubPosition = 150;

    public SlideSubsystem(Motor slideMotor) {
        this.slideMotor = slideMotor;
    }

    public void moveToHub() {
        // set the run mode
        slideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        slideMotor.setPositionCoefficient(0.05);
        double kP = slideMotor.getPositionCoefficient();

        // set the target position
        slideMotor.setTargetPosition(hubPosition); // an integer representing desired tick count

        slideMotor.set(0);

        // set the tolerance
        slideMotor.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!slideMotor.atTargetPosition()) {
            slideMotor.set(0.75);
        }
        slideMotor.stopMotor(); // stop the motor

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

    public void moveToHome() {
        // set the run mode
        slideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        slideMotor.setPositionCoefficient(0.05);
        double kP = slideMotor.getPositionCoefficient();

        // set the target position
        slideMotor.setTargetPosition(homePosition); // an integer representing desired tick count

        slideMotor.set(0);

        // set the tolerance
        slideMotor.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!slideMotor.atTargetPosition()) {
            slideMotor.set(0.75);
        }
        slideMotor.stopMotor(); // stop the motor

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