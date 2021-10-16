package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;

public class DepositCommand extends CommandBase {
    private DepositSubsystem depositSubsystem;

    public DepositCommand(DepositSubsystem subsystem){
        depositSubsystem = subsystem;
        addRequirements(depositSubsystem);
    }

    @Override
    public void initialize(){
        depositSubsystem.depositArmRotation.setPosition(0);
    }
    @Override
    public void execute(){
        depositSubsystem.depositArmRotation.setPosition(depositSubsystem.getDepositPosition());
    }
    @Override
    public void cancel(){
        depositSubsystem.depositArmRotation.setPosition(0);
        CommandScheduler.getInstance().cancel(this);
    }
}
