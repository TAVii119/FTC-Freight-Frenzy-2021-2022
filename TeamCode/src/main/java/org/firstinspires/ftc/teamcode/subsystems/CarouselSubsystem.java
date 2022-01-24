package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class CarouselSubsystem extends SubsystemBase {
    public CRServo duckLeftServo;

    public CarouselSubsystem(CRServo duckLeftServo) {
        this.duckLeftServo = duckLeftServo;
    }

    public void runCarousel() {
        duckLeftServo.setPower(0.5);
    }

    public void stopCarousel() {
        duckLeftServo.setPower(0);
    }
}
