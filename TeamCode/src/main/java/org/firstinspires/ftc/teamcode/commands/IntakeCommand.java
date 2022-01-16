package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSystem;
    public double intake, outtake;

    public IntakeCommand(IntakeSubsystem subsystem, double intakePower, double outtakePower){
       intakeSystem = subsystem;
       intake = intakePower;
       outtake = outtakePower;
       addRequirements(intakeSystem);

    }
    @Override
    public void execute(){
        intakeSystem.runIntake(intake - outtake);
    }
}
