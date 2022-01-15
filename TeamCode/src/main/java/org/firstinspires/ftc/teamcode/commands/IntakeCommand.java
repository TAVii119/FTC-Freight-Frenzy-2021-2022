package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.TeleOperated;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

    /*
    This is our intake command. This works hand in hand with the IntakeSubsystem class.
    Using our DualShock 4 Controllers we control the intake.
    The motors get assigned power from -1 to 1 based on how far we press the triggers.
     */
public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    public DoubleSupplier intake, outtake;
    public static boolean intakeRunning = false;
    public static Timing.Timer intakeTimer;



    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intake,
                         DoubleSupplier outtake) {

        this.intakeSubsystem = intakeSubsystem;
        this.intake = intake;
        this.outtake = outtake;
        intakeTimer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        //if(TeleOperated.intakeTimer.elapsedTime() >=0.04 && TeleOperated.intakeTimer.elapsedTime() <=0.97)
            intakeSubsystem.runIntake(-1);
        //else
            intakeSubsystem.runIntake(0);

        intakeSubsystem.runIntake(intake.getAsDouble() - outtake.getAsDouble());

        if (intake.getAsDouble() > 0.1)
            intakeRunning = true;
        else intakeRunning = false;
    }
}