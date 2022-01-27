package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class CarouselSubsystem extends SubsystemBase {
    private Motor duckRightMotor;
    public boolean isCarouselRunning = false;

    public CarouselSubsystem(Motor duckRightMotor) {
        this.duckRightMotor = duckRightMotor;
    }

    public void runCarousel(){
        duckRightMotor.set(0.75);
        isCarouselRunning = true;
    }

    public void stopCarousel(){
        duckRightMotor.set(0);
        isCarouselRunning = false;
    }
}
