package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RetryNearBlueMec {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity Istanbul = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14.9606, 17.7165)
                .build();

        Istanbul.runAction(Istanbul.getDrive().actionBuilder(new Pose2d(-55.3, -55.2, -90))
                        .strafeToLinearHeading(new Vector2d(-41.4, -40.7), Math.toRadians(-20))
                        .waitSeconds(0.4)

                        .strafeToConstantHeading(new Vector2d(-6.9, -46.4))
                        .setReversed(true)
                        .strafeToConstantHeading(new Vector2d(-41.4, -40.7))
                        .waitSeconds(0.4)

                        .strafeToLinearHeading(new Vector2d(17.1, -46.0), Math.toRadians(5))
                        .setReversed(true)
                        .strafeToConstantHeading(new Vector2d(-23.1, -23.1))
                        .waitSeconds(0.4)

                        .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                        .splineToLinearHeading(new Pose2d(new Vector2d(-4.1, -65.0), 10), 10)
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                        .waitSeconds(0.4)

                        .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                        .splineToLinearHeading(new Pose2d(new Vector2d(-4.1, -65.0), 10), 10)
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                        .waitSeconds(0.4)

                        .strafeToLinearHeading(new Vector2d(37.9, -46.5), Math.toRadians(-15))
                        .setReversed(true)
                        .strafeToConstantHeading(new Vector2d(-24.5, -9.2))
//
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Istanbul)
                .start();
    }
}
