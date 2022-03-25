package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestColorSensor extends LinearOpMode {
    // Define a variable for our color sensor
    RevColorSensorV3 color;
    boolean SensorisActive = false;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(RevColorSensorV3.class, "cupSensor");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            SensorisActive = false;
            telemetry.addData("Distanta", getDistance());
            telemetry.update();

            if(color.getDistance(DistanceUnit.CM) < 4 && SensorisActive == false){
                telemetry.addLine("Element in Cupa");
                gamepad1.rumble(75);
                SensorisActive = true;
                sleep(200);
                gamepad1.stopRumble();
            }
        }
    }
    public double getDistance(){
        return color.getDistance(DistanceUnit.CM);
    }
}
