package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;

public class FourBarCommand extends CommandBase {

    FourBarSubsystem fourBarSubsystem;
    IntakeCommand intakeCommand;

    public FourBarCommand(FourBarSubsystem fourBarSubsystem, IntakeCommand intakeCommand){

        this.fourBarSubsystem = fourBarSubsystem;
        this.intakeCommand = intakeCommand;
        addRequirements(fourBarSubsystem);
    }

    @Override
    public void initialize() {
        fourBarSubsystem.rightServo.setPosition(0);
        fourBarSubsystem.leftServo.setPosition(0);
    }

    @Override
    public void execute() {
        if (intakeCommand.intakeRunning) {
            fourBarSubsystem.setLevelIntake();
        }
        fourBarSubsystem.rightServo.setPosition(fourBarSubsystem.getFourBarPosition());
        fourBarSubsystem.leftServo.setPosition(fourBarSubsystem.getFourBarPosition());
    }
}
