package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;

public class FourBarCommand extends CommandBase {

    FourBarSubsystem fourBarSubsystem;
    public boolean fourBarIntake = false;

    public FourBarCommand(FourBarSubsystem fourBarSubsystem){
        this.fourBarSubsystem = fourBarSubsystem;
        addRequirements(fourBarSubsystem);
    }
}