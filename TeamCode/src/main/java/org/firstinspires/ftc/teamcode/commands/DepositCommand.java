package org.firstinspires.ftc.teamcode.commands;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;

public class DepositCommand extends CommandBase {

    DepositSubsystem depositSubsystem;
    IntakeCommand intakeCommand;

    public DepositCommand(DepositSubsystem depositSubsystem, IntakeCommand intakeCommand){
        this.depositSubsystem = depositSubsystem;
        this.intakeCommand = intakeCommand;

        addRequirements(depositSubsystem);
    }

    @Override
    public void initialize(){
        depositSubsystem.depositServo.setPosition(0);
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