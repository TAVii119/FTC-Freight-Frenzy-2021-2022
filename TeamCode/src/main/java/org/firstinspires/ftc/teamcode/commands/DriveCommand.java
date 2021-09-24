package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

    /*
    This is our drive command. This works hand in hand with the DriveSubsystem class.
    Using our DualShock 4 Controllers we control the chassis.
    The motors get assigned power from -1 to 1 based on how far we push the joysticks.
     */
public class DriveCommand extends CommandBase {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    /**
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_forward.getAsDouble(), m_rotation.getAsDouble());
    }
}

