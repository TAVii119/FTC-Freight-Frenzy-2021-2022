package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class SlideSubsystem extends SubsystemBase {
    public Motor leftSlideMotor;
    public Motor rightSlideMotor;
    private int slideLevel3Pos = 0;
    private int slideLevel2Pos = 0;
    private int slideLevel1Pos = 0;
    private int slideIntermediate = 0;
    private int homePos = 0;

    public SlideSubsystem(Motor leftSlideMotor, Motor rightSlideMotor) {
        this.leftSlideMotor = leftSlideMotor;
        this.rightSlideMotor = rightSlideMotor;
    }

    public void manualControl(double rightTrigger, double leftTrigger) {
        leftSlideMotor.setRunMode(Motor.RunMode.RawPower);
        rightSlideMotor.setRunMode(Motor.RunMode.RawPower);
        leftSlideMotor.set((rightTrigger - leftTrigger) * 0.5);
        rightSlideMotor.set((rightTrigger - leftTrigger) * 0.5);
    }

    public void slideHome() {
        // set the run mode
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        leftSlideMotor.setPositionCoefficient(0.05);
        rightSlideMotor.setPositionCoefficient(0.05);

        double leftkP = leftSlideMotor.getPositionCoefficient();
        double rightkP = rightSlideMotor.getPositionCoefficient();

        // set the target position
        leftSlideMotor.setTargetPosition(homePos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(homePos);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(0.6);
            rightSlideMotor.set(0.6);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }


    public void slideLevel3() {
        // set the run mode
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        leftSlideMotor.setPositionCoefficient(0.05);
        rightSlideMotor.setPositionCoefficient(0.05);

        double leftkP = leftSlideMotor.getPositionCoefficient();
        double rightkP = rightSlideMotor.getPositionCoefficient();

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel3Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel3Pos);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(0.6);
            rightSlideMotor.set(0.6);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }

    public void slideLevel2() {
        // set the run mode
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        leftSlideMotor.setPositionCoefficient(0.05);
        rightSlideMotor.setPositionCoefficient(0.05);

        double leftkP = leftSlideMotor.getPositionCoefficient();
        double rightkP = rightSlideMotor.getPositionCoefficient();

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel2Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel2Pos);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(0.6);
            rightSlideMotor.set(0.6);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }
    public void slideLevel1() {
        // set the run mode
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        leftSlideMotor.setPositionCoefficient(0.05);
        rightSlideMotor.setPositionCoefficient(0.05);

        double leftkP = leftSlideMotor.getPositionCoefficient();
        double rightkP = rightSlideMotor.getPositionCoefficient();

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel1Pos); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideLevel1Pos);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(0.6);
            rightSlideMotor.set(0.6);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }

    public void slideIntermediate() {
        // set the run mode
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        leftSlideMotor.setPositionCoefficient(0.05);
        rightSlideMotor.setPositionCoefficient(0.05);

        double leftkP = leftSlideMotor.getPositionCoefficient();
        double rightkP = rightSlideMotor.getPositionCoefficient();

        // set the target position
        leftSlideMotor.setTargetPosition(slideIntermediate); // an integer representing desired tick count
        rightSlideMotor.setTargetPosition(slideIntermediate);

        leftSlideMotor.set(0);
        rightSlideMotor.set(0);

        // set the tolerance
        leftSlideMotor.setPositionTolerance(13.6);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(13.6);

        // perform the control loop
        while (!leftSlideMotor.atTargetPosition()) {
            leftSlideMotor.set(0.6);
            rightSlideMotor.set(0.6);
        }
        leftSlideMotor.stopMotor();
        rightSlideMotor.stopMotor();// stop the motor
    }




}