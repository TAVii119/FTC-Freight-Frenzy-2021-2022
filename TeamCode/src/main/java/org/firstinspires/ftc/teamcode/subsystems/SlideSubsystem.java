package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SlideSubsystem extends SubsystemBase {
    private DcMotor slideMotor;
    private int homePosition = 0;
    private int hubPosition = 0;

    public SlideSubsystem(DcMotor slideMotor) {
        this.slideMotor = slideMotor;
    }

    public void extendToHub() {
        this.slideMotor.setTargetPosition(hubPosition);
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor.setPower(1);
    }

    public void returnHome() {
        this.slideMotor.setTargetPosition(homePosition);
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor.setPower(1);
    }
}
