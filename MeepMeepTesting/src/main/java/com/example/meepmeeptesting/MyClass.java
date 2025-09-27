package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass
{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        //myBot = zoeRout1(meepMeep);
        myBot = zoeRout2(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }

     static public RoadRunnerBotEntity zoeRout1(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 24,Math.toRadians(180) ))
            .waitSeconds(3)
            .splineTo(new Vector2d(-12, 20), Math.toRadians(135))
            .waitSeconds(3)
            //Shoots first three balls.
            .turnTo(Math.toRadians(90))
            .splineTo(new Vector2d(-12, 45), Math.toRadians(90))
            .setReversed(true)
            .splineTo(new Vector2d(-12, 20), Math.toRadians(135+180))
            .waitSeconds(3)
            .splineTo(new Vector2d(12, 45), Math.toRadians(90))
            .setReversed(true)
            .splineTo(new Vector2d(-12, 20), Math.toRadians(135+180))
            .waitSeconds(3)
            .splineTo(new Vector2d(36, 45), Math.toRadians(90))
            .waitSeconds(3)
            .build());

        return myBot;
    }

    static public RoadRunnerBotEntity zoeRout2(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, 50,Math.toRadians(126) ))
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(-12, 10), Math.toRadians(135+180))
                .waitSeconds(3)
                .splineTo(new Vector2d(-12, 45), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(-12, 10), Math.toRadians(135+180))
                .waitSeconds(3)
                .splineTo(new Vector2d(12, 45), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(-12, 10), Math.toRadians(135+180))
                .waitSeconds(3)
                .splineTo(new Vector2d(36, 45), Math.toRadians(90))
                .waitSeconds(3)

                     // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0,Math.toRadians(0) ))
                     //.setTangent(180)
                     //.splineTo(new Vector2d(48, 48), Math.toRadians(0))
            .build());


        return myBot;
    }
}