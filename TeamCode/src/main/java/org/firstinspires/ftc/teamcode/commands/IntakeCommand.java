package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    public static boolean intakeRunning = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeMotor.set(intakeSubsystem.getIntakePower());

        if (intakeSubsystem.getIntakePower() != 0.0)
            intakeRunning = true;
        else intakeRunning = false;
    }
}