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
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 58, -Math.toRadians(270)))
                                .back(3)
                                .lineToLinearHeading(new Pose2d(-45, 50, Math.toRadians(120)))
                                .lineToLinearHeading(new Pose2d(-55, 53, Math.toRadians(100)))
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