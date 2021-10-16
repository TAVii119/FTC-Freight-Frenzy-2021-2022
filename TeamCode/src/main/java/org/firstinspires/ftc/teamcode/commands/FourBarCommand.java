package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;

public class FourBarCommand extends CommandBase {
    FourBarSubsystem fourBarSubsystem;

    public FourBarCommand(FourBarSubsystem BarSubsystem){
        fourBarSubsystem = BarSubsystem;
    }

    @Override
    public void initialize(){
        fourBarSubsystem.rightServo.setPosition(0);
        fourBarSubsystem.leftServo.setPosition(0);
    }
    @Override
    public void execute(){
        fourBarSubsystem.leftServo.setPosition(fourBarSubsystem.getFourBarPosition());
        fourBarSubsystem.rightServo.setPosition(fourBarSubsystem.getFourBarPosition());
    }
    public void cancel(){
        fourBarSubsystem.setLevel(0);
        CommandScheduler.getInstance().cancel(this);
    }
}
