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
                .setConstraints(60, 60, Math.toRadians(1051), Math.toRadians(360), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12, -62, Math.toRadians(270)))
                                .lineTo(new Vector2d(-12, -42))
                                .lineToLinearHeading(new Pose2d(-57.3, -60, Math.toRadians(190)))
                                .forward(1.3)
                                .back(25)
                                .turn(Math.toRadians(150))
                                .lineToLinearHeading(new Pose2d(10, -63, Math.toRadians(355)))
                                .forward(33)
                                .back(35)
                                .lineToLinearHeading(new Pose2d(-12, -41, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(11, -52, Math.toRadians(360)))
                                .lineToSplineHeading(new Pose2d(11, -63, Math.toRadians(330)))
                                .lineTo(new Vector2d(40, -62))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}