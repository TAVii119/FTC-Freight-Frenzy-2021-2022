package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

    /*
    This is our intake command. This works hand in hand with the IntakeSubsystem class.
    Using our DualShock 4 Controllers we control the intake.
    The motors get assigned power from -1 to 1 based on how far we press the triggers.
     */
public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    public DoubleSupplier intake, outtake;
    public boolean intakeRunning = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intake,
                         DoubleSupplier outtake) {

        this.intakeSubsystem = intakeSubsystem;
        this.intake = intake;
        this.outtake = outtake;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntake(intake.getAsDouble() - outtake.getAsDouble());

        if (intake.getAsDouble() > 0.1)
            intakeRunning = true;
        else intakeRunning = false;
    }
}