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
                        drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(270)))
                                .back(10)
                                .lineToLinearHeading(new Pose2d(4, -42, Math.toRadians(300)))
                                .lineToLinearHeading(new Pose2d(10, -61, Math.toRadians(340)))
                                .lineToLinearHeading(new Pose2d(45, -64, Math.toRadians(360)))
                                .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(370)))
                                .lineToLinearHeading(new Pose2d(4, -41, Math.toRadians(320)))
                                .lineToLinearHeading(new Pose2d(12, -62, Math.toRadians(340)))
                                .lineToLinearHeading(new Pose2d(48, -64, Math.toRadians(360)))
                                .lineToLinearHeading(new Pose2d(10, -64, Math.toRadians(365)))
                                .lineToLinearHeading(new Pose2d(4, -41, Math.toRadians(280)))
                                .lineToLinearHeading(new Pose2d(12, -64, Math.toRadians(335)))
                                .lineToLinearHeading(new Pose2d(48, -66, Math.toRadians(360)))
                                .lineToLinearHeading(new Pose2d(10, -64, Math.toRadians(365)))
                                .lineToLinearHeading(new Pose2d(4, -41, Math.toRadians(280)))
                                .lineToLinearHeading(new Pose2d(12, -65, Math.toRadians(340)))
                                .lineToLinearHeading(new Pose2d(48, -67, Math.toRadians(340)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}