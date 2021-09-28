package org.firstinspires.ftc.teamcode.test.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;


/*
This is our drive subsystem. This subsystem is for our 6 wheel drive chassis with 6 DC Motors.
 */
    public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;
    private final Motor LB;
    private final Motor LF;
    private final Motor RB;
    private final Motor RF;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(Motor leftB, Motor leftF, Motor rightB, Motor rightF ){
        LB=leftB;
        LF=leftF;
        RB=rightB;
        RF=rightF;
        m_drive = new MecanumDrive(LB, LF, RB, RF);
    }


    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     * @param str the commanded strafe
     */
    public void drive(double fwd, double rot, double str) {
        m_drive.driveRobotCentric(str, fwd, rot);
    }

}



