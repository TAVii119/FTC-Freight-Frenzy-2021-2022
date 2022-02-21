package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TSESubsystem;
import org.firstinspires.ftc.teamcode.Timing;

public class ScoreCommand extends CommandBase {
    FourBarSubsystem fourBarSubsystem;
    DepositSubsystem depositSubsystem;
    TSESubsystem tseSubsystem;
    IntakeSubsystem intakeSubsystem;
    IntakeLiftSubsystem intakeLiftSubsystem;

    Timing.Timer scoreTimer;

    public ScoreCommand(FourBarSubsystem fourBarSubsystem, DepositSubsystem depositSubsystem, IntakeSubsystem intakeSubsystem, IntakeLiftSubsystem intakeLiftSubsystem){
        this.fourBarSubsystem = fourBarSubsystem;
        this.depositSubsystem = depositSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.intakeLiftSubsystem = intakeLiftSubsystem;
        addRequirements(fourBarSubsystem, depositSubsystem, intakeSubsystem, intakeLiftSubsystem);
    }

    @Override
    public void execute() {
        depositSubsystem.closeDeposit();
        intakeSubsystem.reverseIntake();
        intakeLiftSubsystem.iLifterfourBarPos();
        fourBarSubsystem.fourBarTopPos();
        cancel();
    }
}
