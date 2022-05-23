package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 9.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11, -61.63, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(55, -64, Math.toRadians(360)))
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(22, -66, Math.toRadians(360)))
                                .splineTo(new Vector2d(-5, -46), -Math.toRadians(245))

                               /* .setReversed(false)
                                .splineToSplineHeading(new Pose2d(8, 60, -Math.toRadians(360)), Math.toRadians(360))
                                .lineToLinearHeading(new Pose2d(40, 60, -Math.toRadians(360)))
                                .setReversed(true)
                                .splineTo(new Vector2d(5, 40), -Math.toRadians(110))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(8, 60, -Math.toRadians(360)), Math.toRadians(360))
                                .lineToLinearHeading(new Pose2d(40, 60, -Math.toRadians(360)))
                                .setReversed(true)
                                .splineTo(new Vector2d(5, 40), -Math.toRadians(110))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(8, 60, -Math.toRadians(360)), Math.toRadians(360))
                                .lineToLinearHeading(new Pose2d(40, 60, -Math.toRadians(360)))
                                .setReversed(true)
                                .splineTo(new Vector2d(5, 40), -Math.toRadians(110))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(8, 60, -Math.toRadians(360)), Math.toRadians(360))
                                .lineToLinearHeading(new Pose2d(40, 60, -Math.toRadians(360)))
                                .setReversed(true)
                                .splineTo(new Vector2d(5, 40), -Math.toRadians(110))
                                .setReversed(false)*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}