package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class CarouselSubsystem extends SubsystemBase {
    public CRServo duckLeftServo;
    public CRServo duckRightServo;
    public boolean carouselRunning = false;
    private double carouselPower;

    public CarouselSubsystem(CRServo duckLeftServo, CRServo duckRightServo) {
        this.duckRightServo = duckRightServo;
        this.duckLeftServo = duckLeftServo;
    }

    public void startCarousel(){
        this.carouselPower = 1;
    }
    public void stopCarousel() {
       this.carouselPower = 0;
    }
    public double getCarouselPower() {
        return this.carouselPower;
    }
}