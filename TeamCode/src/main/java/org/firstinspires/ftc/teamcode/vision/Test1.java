package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(name = "New Barcode Test", group = "Test")
public class Test1 extends LinearOpMode {
    BarcodeUtil webcamUtil;
    BarCodeDetection.BarcodePosition TSEPosition;

    @Override
    public void runOpMode( ) {
        webcamUtil = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, 1);
        webcamUtil.init();

        waitForStart();
        while(opModeIsActive()){
            TSEPosition = webcamUtil.getBarcodePosition();
            telemetry.addData("<Position", TSEPosition);
        }
    }
}