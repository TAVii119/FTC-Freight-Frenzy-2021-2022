package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TimerTest")

public class TimerTest extends CommandOpMode {

    Timing.Timer scoreTimer;

    private Motor duckMotor;

    private InstantCommand startMotor;

    GamepadEx driver1;

    @Override
    public void initialize() {

        duckMotor = new Motor(hardwareMap, "duckMotor");
        driver1 = new GamepadEx(gamepad1);
        scoreTimer = new Timing.Timer(5);

        startMotor = new InstantCommand(() -> {
            scoreTimer.start();

            while (!scoreTimer.done()) {}

            duckMotor.set(1);

            scoreTimer.pause();
        });


        Button muie = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(startMotor);
    }
}
