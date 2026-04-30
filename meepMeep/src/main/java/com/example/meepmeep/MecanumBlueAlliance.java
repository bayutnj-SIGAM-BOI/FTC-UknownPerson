package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MecanumBlueAlliance {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity Istanbul = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14.9606, 17.7165)
                .build();

        Istanbul.runAction(Istanbul.getDrive().actionBuilder(new Pose2d(-55.3, -55.2, Math.toRadians(-90)))
                        .strafeToLinearHeading(new Vector2d(-16.0, -16.4), Math.toRadians(-45))
                        .waitSeconds(0.6)

                        .strafeToLinearHeading(new Vector2d(13.4, -47.8), 30)
                        .waitSeconds(0.3)
                        .strafeToConstantHeading(new Vector2d(-16.0, -16.4))
                        .waitSeconds(0.6)

                        .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                        .splineToLinearHeading(new Pose2d(new Vector2d(-4.1, -65.0), 10), 10)
                        .waitSeconds(3)

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                        .waitSeconds(0.6)

                        .setTangent(5)
                        .strafeToLinearHeading(new Vector2d(-12.0, -34.5),Math.toRadians(-90))
                        .strafeToConstantHeading(new Vector2d(-12.0, -55.0))
//
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-23.3, -23.4), 8)
                        .waitSeconds(0.6)

                        .strafeToConstantHeading(new Vector2d(34.4, -33.1))
                        .strafeToConstantHeading(new Vector2d(34.7, -52.3))

                        .setReversed(true)
                        .strafeToConstantHeading(new Vector2d(-24.5, -9.2))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Istanbul)
                .start();
    }
}
