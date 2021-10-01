package org.firstinspires.ftc.teamcode.test.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.test.Subsystems.IntakeSubsystem;

/*
This is our intake command. This works hand in hand with the IntakeSubsystem class.
Using our DualShock 4 Controllers we control the intake.
The motors get assigned power from -1 to 1 based on how far we press the triggers.
*/
public class IntakeCommand extends CommandBase {

private final IntakeSubsystem intakeSystem;
private double intake, outtake;

public IntakeCommand(IntakeSubsystem subsystem, double intakePower, double outtakePower) {
    intakeSystem = subsystem;
    intake = intakePower;
    outtake = outtakePower;
    addRequirements(intakeSystem);
}

{
    }

    @Override
public void execute() {
    intakeSystem.runIntake(intake - outtake);
}

@Override
public void cancel() {
    intakeSystem.stopIntake();
    CommandScheduler.getInstance().cancel(this);
}
}