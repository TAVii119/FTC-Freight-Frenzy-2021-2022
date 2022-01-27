package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;

public class FourBarCommand extends CommandBase {

    FourBarSubsystem fourBarSubsystem;

    public FourBarCommand(FourBarSubsystem fourBarSubsystem){

        this.fourBarSubsystem = fourBarSubsystem;
        addRequirements(fourBarSubsystem);
    }

    @Override
    public void execute() {
        fourBarSubsystem.rightServo.setPosition(fourBarSubsystem.getFourBarPosition());
        fourBarSubsystem.leftServo.setPosition(fourBarSubsystem.getFourBarPosition());
    }
}
