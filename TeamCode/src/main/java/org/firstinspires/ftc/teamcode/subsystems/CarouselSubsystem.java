package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class CarouselSubsystem extends SubsystemBase {
    private Motor duckMotor;

    public CarouselSubsystem(Motor duckMotor) {
        this.duckMotor = duckMotor;
    }

    public void runCarousel() {
        duckMotor.resetEncoder();
        duckMotor.setRunMode(Motor.RunMode.RawPower);

        while (duckMotor.getCurrentPosition() < 700)
            duckMotor.set(0.3);

        while (duckMotor.getCurrentPosition() < 1200)
            duckMotor.set(1);

        duckMotor.stopMotor();
    }
}
