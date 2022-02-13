package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(980);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(1051), Math.toRadians(360), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12, -62, Math.toRadians(270)))
                                .lineTo(new Vector2d(-12, -42))
                                .lineToSplineHeading(new Pose2d(-61, -57, Math.toRadians(190)))
                                .forward(2)
                                .back(30)
                                .splineToSplineHeading(new Pose2d(15, -68), Math.toRadians(0))
                                .lineTo(new Vector2d(37, -68))
                                .back(40)
                                .lineToSplineHeading(new Pose2d(-12, -37, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(5, -68, Math.toRadians(360)))
                                .lineTo(new Vector2d(39, -68))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}