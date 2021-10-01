package org.firstinspires.ftc.teamcode.test.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.test.Subsystems.ArmSubsystem;


public class ArmCommand extends CommandBase {

    // The subsystem the command runs on
    private ArmSubsystem armSubsystem;

    public ArmCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
    }

    @Override
    public void execute() {
        armSubsystem.LeftarmRotation.setPosition(armSubsystem.getCurrentPosition());
        armSubsystem.RightarmRotation.setPosition(armSubsystem.getCurrentPosition());
    }

    @Override
    public void cancel()
    {
        armSubsystem.RightarmRotation.setPosition(0);
        armSubsystem.LeftarmRotation.setPosition(0);
    }

}