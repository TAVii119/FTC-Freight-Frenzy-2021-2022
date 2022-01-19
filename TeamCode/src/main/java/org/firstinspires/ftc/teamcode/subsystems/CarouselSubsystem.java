package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class CarouselSubsystem extends SubsystemBase {
    public CRServo carouselServo;
    public boolean carouselRunning = false;
    private double carouselPower;

    public CarouselSubsystem(CRServo carouselServo) {

        this.carouselServo = carouselServo;
    }

    public void startCarousel(){
        this.carouselPower = 0.9;
    }
    public void stopCarousel() {
       this.carouselPower = 0;
    }
    public double getCarouselPower() {
        return this.carouselPower;
    }
}