package org.firstinspires.ftc.teamcode.commands;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
public class DepositCommand extends CommandBase {
    DepositSubsystem depositSubsystem;

    public DepositCommand(DepositSubsystem depositSubsystem){
        this.depositSubsystem = depositSubsystem;
        addRequirements(depositSubsystem);
    }
}