package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;

public class CarouselCommand extends CommandBase {
    private CarouselSubsystem carouselSubsystem;
    public static boolean carouseRunning = false;

    public CarouselCommand(CarouselSubsystem carouselSubsystem) {
        this.carouselSubsystem = carouselSubsystem;
        addRequirements(carouselSubsystem);
    }
        @Override
        public void execute() {
            carouselSubsystem.carouselServo.setPower(carouselSubsystem.getCarouselPower());

            if (carouselSubsystem.getCarouselPower() != 0.1)
                carouseRunning = true;
            else carouseRunning = false;
    }
}
