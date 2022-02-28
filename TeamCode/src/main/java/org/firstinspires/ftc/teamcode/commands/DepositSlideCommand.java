package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Timing;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class DepositSlideCommand extends CommandBase {
    private DistanceSensor distanceSensor;
    private IntakeSubsystem intakeSubsystem;
    private SlideSubsystem slideSubsystem;
    private DepositSubsystem depositSubsystem;
    private FourBarSubsystem fourBarSubsystem;
    private Timing.Timer scoreTimer;

    public DepositSlideCommand(IntakeSubsystem intakeSubsystem, SlideSubsystem slideSubsystem, DepositSubsystem depositSubsystem, FourBarSubsystem fourBarSubsystem, DistanceSensor distanceSensor) {
        this.intakeSubsystem = intakeSubsystem;
        this.slideSubsystem = slideSubsystem;
        this.depositSubsystem = depositSubsystem;
        this.fourBarSubsystem = fourBarSubsystem;
        this.distanceSensor = distanceSensor;

        addRequirements(intakeSubsystem, slideSubsystem, depositSubsystem, fourBarSubsystem);
    }

    @Override
    public void execute() {
        if(distanceSensor.getDistance(DistanceUnit.CM) <= 3.5 && !slideSubsystem.slideMoving) {
            slideSubsystem.slideMoving = true;
            depositSubsystem.closeDeposit();

            scoreTimer = new Timing.Timer(100);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }

            fourBarSubsystem.fourBarIntermediate();
            intakeSubsystem.reverseIntake();
            slideSubsystem.slideTop();
            fourBarSubsystem.fourBarTop();
        }
    }
}
