package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarCodeDetection extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat( );

    public enum BarcodePosition {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    private BarcodePosition barcodePosition;

    static final Rect LEFT_ROW = new Rect(
            new Point( 476, 357 ),
            new Point( 580, 471 ) );
    static final Rect MIDDLE_ROW = new Rect(
            new Point( 638, 433 ),
            new Point( 763, 575 ) );
    static final Rect RIGHT_ROW = new Rect(
            new Point( 878, 566 ),
            new Point( 1029, 713 ) );

    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public BarCodeDetection(Telemetry t ) {
        telemetry = t;
    }

    public Mat processFrame( Mat input, String type ) {

        Imgproc.cvtColor( input, mat, Imgproc.COLOR_RGB2HSV );
        Scalar lowHSV;
        Scalar highHSV;

        /*if( type.equalsIgnoreCase( "TSE" ) ) {
            lowHSV = new Scalar( 25, 25, 35 );
            highHSV = new Scalar( 40, 255, 255 );
        } else {
            lowHSV = new Scalar( 40, 50, 70 );
            highHSV = new Scalar( 65, 255, 255 );
        }*/
        lowHSV = new Scalar(10, 100, 20);
        highHSV = new Scalar(25, 255, 255);
        Core.inRange( mat, lowHSV, highHSV, mat );

        Mat left = mat.submat(LEFT_ROW);
        Mat middle = mat.submat(MIDDLE_ROW);
        Mat right = mat.submat(RIGHT_ROW);

        double leftValue = Core.sumElems( left ).val[0] / LEFT_ROW.area( ) / 255;
        double middleValue = Core.sumElems( middle ).val[0] / MIDDLE_ROW.area( ) / 255;
        double rightValue = Core.sumElems( right ).val[0] / RIGHT_ROW.area( ) / 255;

        left.release( );
        middle.release( );
        right.release( );

        boolean leftBool = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean middleBool = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean rightBool = rightValue > PERCENT_COLOR_THRESHOLD;

        if( rightBool ) {
            barcodePosition = BarcodePosition.RIGHT;
            telemetry.addData( "Location", type + " right" );
        } else if( leftBool ) {
            barcodePosition = BarcodePosition.LEFT;
            telemetry.addData( "Location", type + " left" );
        } else if( middleBool ) {
            barcodePosition = BarcodePosition.MIDDLE;
            telemetry.addData( "Location", type + " middle" );
        } else {
            barcodePosition = BarcodePosition.NOT_FOUND;
            telemetry.addData( "Location", type + " not found" );
        }
        Imgproc.cvtColor( mat, mat, Imgproc.COLOR_GRAY2RGB );

        Scalar elementColor = new Scalar( 255, 0, 0 );
        Scalar notElement = new Scalar( 0, 255, 0 );

        Imgproc.rectangle( mat, LEFT_ROW, barcodePosition == BarcodePosition.LEFT ? notElement : elementColor );
        Imgproc.rectangle( mat, RIGHT_ROW, barcodePosition == BarcodePosition.RIGHT ? notElement : elementColor );
        Imgproc.rectangle( mat, MIDDLE_ROW, barcodePosition == BarcodePosition.MIDDLE ? notElement : elementColor );
        return mat;
    }

    @Override
    public Mat processFrame( Mat input ) {

        Mat elementImage = processFrame( input, "element" );
        Mat duckImage = processFrame( input, "duck" );
        double elementValue = Core.sumElems( elementImage ).val[0] / (elementImage.rows( ) * elementImage.cols( )) / 255;
        double duckValue = Core.sumElems( duckImage ).val[0] / (duckImage.rows( ) * duckImage.cols( )) / 255;
        telemetry.update( );
        if( elementValue < duckValue )
            return duckImage;
        return elementImage;
    }

    public BarcodePosition getBarcodePosition( ) {
        return barcodePosition;
    }
}