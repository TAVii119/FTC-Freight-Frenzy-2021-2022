package org.firstinspires.ftc.teamcode.test.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.test.Subsystems.ArmSubsystem;


public class ArmCommand extends CommandBase {

    // The subsystem the command runs on
    private ArmSubsystem m_armSubsystem;

    public void rotate(ArmSubsystem subsystem) {
        m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.rotate();
    }

    @Override
    public void cancel()
    {   m_armSubsystem.release();

    }

}