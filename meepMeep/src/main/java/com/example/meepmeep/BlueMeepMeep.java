package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity Go2Steam = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Go2Steam.runAction(Go2Steam.getDrive().actionBuilder(new Pose2d(-55.5, -55.3, Math.toRadians(45)))
                .lineToX(-28.2)
                .waitSeconds(1.6)

                .turn(Math.toRadians(-95))
                .lineToX(-10.3)
                .lineToX(-28.2)
                .waitSeconds(1.6)

                .turn(Math.toRadians(25))
                .lineToX(12.9)
                .lineToX(-27.7)
                .waitSeconds(1.6)
//
                .turn(Math.toRadians(-25))
                .lineToX(-4.7)
                .lineToX(-27.7)
//
//                .turn(Math.toRadians(35))
//                .lineToX(34.9)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Go2Steam)
                .start();
    }
}