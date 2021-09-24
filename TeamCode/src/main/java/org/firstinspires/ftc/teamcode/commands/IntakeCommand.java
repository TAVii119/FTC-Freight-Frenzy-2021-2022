package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSystem;
    private double intake, outtake;

    public IntakeCommand(IntakeSubsystem subsystem, double intakePower, double outtakePower) {
        intakeSystem = subsystem;
        intake = intakePower;
        outtake = outtakePower;
        addRequirements(intakeSystem);
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