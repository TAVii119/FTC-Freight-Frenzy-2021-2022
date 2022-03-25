package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors extends SubsystemBase {

    //forward sensors
    private  Rev2mDistanceSensor forwardSensor;
    //The TOF Distance sensor on the sides.
    private  Rev2mDistanceSensor leftSensor;
    private  Rev2mDistanceSensor rightSensor;

    //Debugging rate timer
    private final ElapsedTime cycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double cycleTime = 0.0;

    public DistanceSensors(HardwareMap hardwareMap) {
        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensorFront");
        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensorLeft");
        rightSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor");

//        disableAll();

    }


//    public void enableAll(){
//        leftSensor.enable();
//        forwardSensor.enable();
//        backwardSensor.enable();
//    }
//
//    public void disableAll(){
//        leftSensor.disable();
//        forwardSensor.disable();
//        backwardSensor.disable();
//    }

    @Override
    public void periodic() {
        cycleTime = cycleTimer.milliseconds();
        cycleTimer.reset();
    }


    //Get the left sensor range in cm
    public double getLeftRange(DistanceUnit unit) {
        return unit.fromCm(leftSensor.getDistance(DistanceUnit.CM));
    }

    public double getForwardRange(DistanceUnit unit) {
        return unit.fromCm(forwardSensor.getDistance(DistanceUnit.CM));
    }

    public double getBackwardRange(DistanceUnit unit) {
        return unit.fromCm(rightSensor.getDistance(DistanceUnit.CM));
    }

    /**
     * Returns the cycle time in milliseconds of the distance sensors.
     */
    public double getCycleTime() {
        return cycleTime;
    }

    //Tests the sensor by running a range command and seeing if there's an output, takes at least 100ms
    public boolean test() {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 80) ;
        return forwardSensor.getDistance(DistanceUnit.CM) > 20 || //20cm is the minimum range, so we test with it
                leftSensor.getDistance(DistanceUnit.CM) > 3 ||
                rightSensor.getDistance(DistanceUnit.CM) > 3;
    }
}