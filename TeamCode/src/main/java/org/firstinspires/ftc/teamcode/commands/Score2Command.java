package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TSESubsystem;
import org.firstinspires.ftc.teamcode.Timing;

public class Score2Command extends CommandBase {
    FourBarSubsystem fourBarSubsystem;
    DepositSubsystem depositSubsystem;
    TSESubsystem tseSubsystem;
    IntakeSubsystem intakeSubsystem;
    IntakeLiftSubsystem intakeLiftSubsystem;

    Timing.Timer scoreTimer;

    public Score2Command(FourBarSubsystem fourBarSubsystem, DepositSubsystem depositSubsystem, IntakeSubsystem intakeSubsystem, IntakeLiftSubsystem intakeLiftSubsystem){
        this.fourBarSubsystem = fourBarSubsystem;
        this.depositSubsystem = depositSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.intakeLiftSubsystem = intakeLiftSubsystem;
        addRequirements(fourBarSubsystem, depositSubsystem,intakeSubsystem, intakeLiftSubsystem);
    }

    @Override
    public void execute() {
        depositSubsystem.pushDeposit();
        // Wait for minerals to be ejected from deposit
        scoreTimer = new Timing.Timer(350);
        scoreTimer.start();
        while (!scoreTimer.done())
        {
            // Wait for timer to end
        }
        scoreTimer.pause();
        fourBarSubsystem.fourBarIntakePos();
        intakeSubsystem.runIntake();
        depositSubsystem.openDeposit();
        scoreTimer = new Timing.Timer(650);
        scoreTimer.start();
        while (!scoreTimer.done())
        {
            // Wait for timer to end
        }
        scoreTimer.pause();
        intakeLiftSubsystem.lifterIntakePos();
        cancel();
    }
}
