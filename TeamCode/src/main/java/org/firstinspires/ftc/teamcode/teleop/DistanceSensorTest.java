package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="DistanceSensorTest", group="Test")
public class DistanceSensorTest extends LinearOpMode {

    DistanceSensor distanceSensorLeft;
    DistanceSensor distanceSensorRight;
    DistanceSensor distanceSensorFront;
    NormalizedColorSensor cupSensor;

    BNO055IMU imu;

    Orientation angles;


    @Override
    public void runOpMode() {

        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceSensorFront");
        cupSensor = hardwareMap.get(NormalizedColorSensor.class, "cupSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {
            NormalizedRGBA colors = cupSensor.getNormalizedColors();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            if (cupSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) cupSensor).getDistance(DistanceUnit.CM));
            }
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addData("distanceSensorLeft", distanceSensorLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("distanceSensorRight", distanceSensorRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("distanceSensorFront", distanceSensorFront.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
