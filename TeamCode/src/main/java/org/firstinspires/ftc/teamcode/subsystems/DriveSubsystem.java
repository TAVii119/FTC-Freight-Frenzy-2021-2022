package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;


    /*
    This is our drive subsystem. This subsystem is for our 6 wheel drive chassis with 6 DC Motors.
     */
    public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(Motor rightBack, Motor leftBack, Motor rightFront, Motor leftFront) {
        m_drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */

    public void drive(double strafe, double fwd, double rot) {
        m_drive.driveRobotCentric(strafe, fwd, rot);
    }
}



