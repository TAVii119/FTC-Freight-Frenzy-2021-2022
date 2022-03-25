package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 9.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -58, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(5, -36.5, Math.toRadians(128)))
                                .splineToSplineHeading(new Pose2d(20, -60, Math.toRadians(360)), Math.toRadians(360))
                                .splineToLinearHeading(new Pose2d(49, -60, Math.toRadians(380)), -Math.toRadians(360))
                                .setReversed(true)
                                .splineTo(new Vector2d(-5, -37), Math.toRadians(90))

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