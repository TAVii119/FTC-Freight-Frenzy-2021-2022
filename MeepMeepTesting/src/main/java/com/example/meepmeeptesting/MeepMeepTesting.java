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
                        drive.trajectorySequenceBuilder(new Pose2d(12, -58, Math.toRadians(270)))
                                .back(10)
                                .lineToLinearHeading(new Pose2d(0, -40, Math.toRadians(310)))
                                .lineToLinearHeading(new Pose2d(12, -55.5, Math.toRadians(340)))
                                .lineToLinearHeading(new Pose2d(46, -58, Math.toRadians(360)))
                                .lineToLinearHeading(new Pose2d(12, -56.5, Math.toRadians(363)))
                                .lineToLinearHeading(new Pose2d(0, -39.5, Math.toRadians(305)))
                                .lineToLinearHeading(new Pose2d(13, -55.5, Math.toRadians(340)))
                                .lineToLinearHeading(new Pose2d(55.5, -57, Math.toRadians(370)))
                                .lineToLinearHeading(new Pose2d(14, -58, Math.toRadians(365)))
                                .lineToLinearHeading(new Pose2d(0, -39.5, Math.toRadians(305)))
                                .lineToLinearHeading(new Pose2d(13, -56.25, Math.toRadians(350)))
                                .lineToLinearHeading(new Pose2d(56, -57, Math.toRadians(350)))
                                .lineToLinearHeading(new Pose2d(13, -56.5, Math.toRadians(365)))
                                .lineToLinearHeading(new Pose2d(0, -39.8, Math.toRadians(300)))
                                .lineToLinearHeading(new Pose2d(12, -55, Math.toRadians(350)))
                                .lineToLinearHeading(new Pose2d(58, -56 , Math.toRadians(350)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}