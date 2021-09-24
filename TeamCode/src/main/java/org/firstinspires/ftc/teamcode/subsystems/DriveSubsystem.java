package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

    public class DriveSubsystem extends SubsystemBase {

    private final DifferentialDrive m_drive;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(MotorGroup leftSideMotors, MotorGroup rightSideMotors) {
        m_drive = new DifferentialDrive(leftSideMotors, rightSideMotors);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void drive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }
}



