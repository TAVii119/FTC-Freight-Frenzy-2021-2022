package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class CarouselSubsystem extends SubsystemBase {
    private Motor duckMotor;
    public boolean isCarouselRunning = false;

    public CarouselSubsystem(Motor duckMotor) {
        this.duckMotor = duckMotor;
    }

    public void runCarousel(){
        duckMotor.set(0.65);
        isCarouselRunning = true;
    }

    public void stopCarousel(){
        duckMotor.set(0);
        isCarouselRunning = false;
    }
    public void powerCarousel(){
        duckMotor.set(1);
        isCarouselRunning = true;
    }
}
