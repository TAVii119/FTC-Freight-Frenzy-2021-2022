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
                        drive.trajectorySequenceBuilder(new Pose2d(12, -58, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(0, -39, Math.toRadians(310)))
                                .splineToSplineHeading(new Pose2d(8, -60, Math.toRadians(360)), Math.toRadians(360))
                                .waitSeconds(0.01)
                                .lineToLinearHeading(new Pose2d(40, -60, Math.toRadians(360)))
                                .lineToLinearHeading(new Pose2d(57, -57, Math.toRadians(380)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}