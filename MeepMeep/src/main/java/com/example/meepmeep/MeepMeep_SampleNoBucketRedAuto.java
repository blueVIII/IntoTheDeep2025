package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_SampleNoBucketRedAuto {
    public static void main(String[] args) {

        System.out.println("Red team stands below window, Blue team stands above window.");

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -62, 0)) //either have heading or turn 90 (prefer)
                .turn(Math.toRadians(90)) //correct positioning
                .turn(Math.toRadians(15))
                .lineToY(-31)
                .lineToY(-40)
                .turn(Math.toRadians(-90))
                .lineToX(40)
                .turn(Math.toRadians(183))
                .lineToX(-57) // drop off @ bucket
                .turn(Math.toRadians(-182))
                .lineToX(52)
                .turn(Math.toRadians(180))
                .lineToX(-56) // drop off @ bucket
                .turn(Math.toRadians(-182))
                .lineToX(64)
                .turn(Math.toRadians(180))
                .lineToX(-53)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}