package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SlideSubsystem extends SubsystemBase {
    public Motor slideMotor;
    private int level3HubPosition = 2280;
    private int level2HubPosition = 0;
    private int level1HubPosition = 0;
    private int homePosition = 0;

    public SlideSubsystem(Motor slideMotor) {
        this.slideMotor = slideMotor;
    }

    public void moveSlideToHubLevel3() {
        // set the run mode
        slideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        slideMotor.setPositionCoefficient(0.05);
        double kP = slideMotor.getPositionCoefficient();

        // set the target position
        slideMotor.setTargetPosition(level3HubPosition); // an integer representing desired tick count

        slideMotor.set(0);

        // set the tolerance
        slideMotor.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!slideMotor.atTargetPosition()) {
            slideMotor.set(0.75);
        }
        slideMotor.stopMotor(); // stop the motor
    }

        public void moveSlideToHubLevel2 () {
            // set the run mode
            slideMotor.setRunMode(Motor.RunMode.PositionControl);

            // set and get the position coefficient
            slideMotor.setPositionCoefficient(0.05);
            double kP = slideMotor.getPositionCoefficient();

            // set the target position
            slideMotor.setTargetPosition(level2HubPosition); // an integer representing desired tick count

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

        public void moveSlideToHubLevel1 () {
            // set the run mode
            slideMotor.setRunMode(Motor.RunMode.PositionControl);

            // set and get the position coefficient
            slideMotor.setPositionCoefficient(0.05);
            double kP = slideMotor.getPositionCoefficient();

            // set the target position
            slideMotor.setTargetPosition(level1HubPosition); // an integer representing desired tick count

            slideMotor.set(0);

            // set the tolerance
            slideMotor.setPositionTolerance(13.6);   // allowed maximum error

            // perform the control loop
            while (!slideMotor.atTargetPosition()) {
                slideMotor.set(0.75);
            }
            slideMotor.stopMotor(); // stop the motor
        }

    public void moveSlideToHome() {
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