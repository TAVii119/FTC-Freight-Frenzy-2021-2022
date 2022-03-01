package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 10.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -58, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-12, -47))
                                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(330)))
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
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}