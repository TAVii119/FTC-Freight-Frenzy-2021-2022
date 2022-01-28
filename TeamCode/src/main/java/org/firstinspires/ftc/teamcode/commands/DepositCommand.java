package org.firstinspires.ftc.teamcode.commands;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;

public class DepositCommand extends CommandBase {
    DepositSubsystem depositSubsystem;

    public DepositCommand(DepositSubsystem depositSubsystem){
        this.depositSubsystem = depositSubsystem;

        addRequirements(depositSubsystem);
    }

    @Override
    public void execute() {
        depositSubsystem.depositServo.setPosition(depositSubsystem.getDepositPosition());
    }

    @Override
    public void cancel() {
        depositSubsystem.depositServo.setPosition(0);
        CommandScheduler.getInstance().cancel(this);
    }
}