package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class SlideCommand extends CommandBase {
    private SlideSubsystem slideSubsystem;

    public SlideCommand(SlideSubsystem slideSubsystem) {
        this.slideSubsystem = slideSubsystem;
    }

    @Override
    public void initialize() {
        //this.slideSubsystem.returnHome();
    }
}