package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SlideSubsystem extends SubsystemBase {
    public DcMotor slideMotor;
    private int slideIntakePosition = 0;
    public boolean slideRunning = false;

    public SlideSubsystem(DcMotor slideMotor){
        this.slideMotor = slideMotor;
    }

    public void extendSlideToScore(int position) {
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor.setTargetPosition(position);
        this.slideMotor.setPower(1);
        this.slideRunning = true;
    }


    public void bringSlideBack() {
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor.setTargetPosition(this.slideIntakePosition);
        this.slideMotor.setPower(1);
        this.slideRunning = true;
    }


    public void stopSlide() {
        this.slideMotor.setPower(0);
        this.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slideRunning = false;
    }
}
