package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

import java.util.function.DoubleSupplier;

public class SlideCommand extends CommandBase {

    private SlideSubsystem slideSubsystem;
    private final DoubleSupplier rightTrigger;
    private final DoubleSupplier leftTrigger;

    public SlideCommand(SlideSubsystem slideSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        this.slideSubsystem = slideSubsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        addRequirements(slideSubsystem);
    }

    @Override
    public void execute() {
        slideSubsystem.manualControl(rightTrigger.getAsDouble(), leftTrigger.getAsDouble());
    }
}