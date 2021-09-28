package org.firstinspires.ftc.teamcode.test.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;



public class ArmSubsystem extends SubsystemBase {

    private final Servo armRotation;

    public ArmSubsystem(Servo ArmRotation)
    {armRotation=ArmRotation;

    }


    /**
     * Rotate arm;
     */
    public void rotate() {
        armRotation.setPosition(0.5);
    }

    /**
     * goes back.
     */
    public void release() {
        armRotation.setPosition(0);
    }}
