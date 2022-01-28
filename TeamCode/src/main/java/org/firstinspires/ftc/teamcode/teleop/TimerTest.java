package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TimerTest")

public class TimerTest extends CommandOpMode {

    Timing.Timer scoreTimer;

    private Motor duckMotor;

    private InstantCommand startMotor;

    @Override
    public void initialize() {

        duckMotor = new Motor(hardwareMap, "duckMotor");

        startMotor = new InstantCommand(() -> {
            scoreTimer.start();

            while (scoreTimer.elapsedTime() < 5) {
            }

            duckMotor.set(1);

            scoreTimer.pause();
        });

    }
}
