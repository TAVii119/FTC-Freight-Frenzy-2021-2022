package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousTest")
public class Autonomie extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence traj1;
    Pose2d startPose;

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-35, -58, Math.toRadians(270));

        waitForStart();
        while(opModeIsActive()) {
            drive.setPoseEstimate(startPose);
            traj1 = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(-12, -47))
                    .lineToLinearHeading(new Pose2d(10, -60.5, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(370)))
                    .lineToLinearHeading(new Pose2d(4, -44, Math.toRadians(300)))

                    .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(335)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(380)))
                    .lineToLinearHeading(new Pose2d(4, -44, Math.toRadians(300)))

                    .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(380)))
                    .lineToLinearHeading(new Pose2d(4, -44, Math.toRadians(300)))
                    .lineToLinearHeading(new Pose2d(10, -60.5, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(48, -62, Math.toRadians(360)))
                    .build();

            drive.followTrajectorySequence(traj1);
            sleep(30000);
        }
    }
}
