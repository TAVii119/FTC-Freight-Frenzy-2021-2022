package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class FourBarCommand extends CommandBase {

    FourBarSubsystem fourBarSubsystem;
    SlideSubsystem slideSubsystem;
    IntakeCommand intakeCommand;

    public FourBarCommand(FourBarSubsystem fourBarSubsystem, SlideSubsystem slideSubsystem, IntakeCommand intakeCommand){
        this.slideSubsystem = slideSubsystem;
        this.fourBarSubsystem = fourBarSubsystem;
        this.intakeCommand = intakeCommand;
        addRequirements(fourBarSubsystem, slideSubsystem);
    }

    @Override
    public void initialize() {
        fourBarSubsystem.barServo.setPosition(0);
    }

    @Override
    public void execute() {
        if (intakeCommand.intakeRunning && !slideSubsystem.slideRunning) {
            fourBarSubsystem.setLevelIntake();
        }
        fourBarSubsystem.barServo.setPosition(fourBarSubsystem.getBarPosition());
    }
}
