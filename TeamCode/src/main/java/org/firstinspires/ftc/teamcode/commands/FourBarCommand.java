package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;

public class FourBarCommand extends CommandBase {

    FourBarSubsystem fourBarSubsystem;

    public FourBarCommand(FourBarSubsystem fourBarSubsystem){

        this.fourBarSubsystem = fourBarSubsystem;
        addRequirements(fourBarSubsystem);
    }

    @Override
    public void initialize() {
        fourBarSubsystem.gbServoRight.setPosition(0);
        fourBarSubsystem.gbServoLeft.setPosition(0);
    }

    @Override
    public void execute() {
        fourBarSubsystem.gbServoRight.setPosition(fourBarSubsystem.getFourBarPosition());
        fourBarSubsystem.gbServoLeft.setPosition(fourBarSubsystem.getFourBarPosition());
    }
}