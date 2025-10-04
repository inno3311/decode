package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass
{
    static int isRed = -1;
    static int isRedRotate = 180;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        myBot = red1rout(meepMeep);
        myBot2 = red2rout(meepMeep);
        //myBot = zoeRout2(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .addEntity(myBot2)
            .start();
    }

     static public RoadRunnerBotEntity zoeRout1(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60,  24, Math.toRadians(180) ))
            .waitSeconds(3)
            .splineTo(new Vector2d(-12,  20), Math.toRadians(135))
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

    static public RoadRunnerBotEntity zoeRout1Blue(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .setColorScheme(new ColorSchemeBlueDark())
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -24, Math.toRadians(180) ))
            .waitSeconds(3)
            .splineTo(new Vector2d(-12, -20), Math.toRadians(135+90))
            .waitSeconds(3)
            //Shoots first three balls.
            .turnTo(Math.toRadians(90+180))
            .splineTo(new Vector2d(-12, -45), Math.toRadians(90+180))
            .setReversed(true)
            .splineTo(new Vector2d(-12, -20), Math.toRadians(135-90))
            .waitSeconds(3)
            .splineTo(new Vector2d(12, -45), Math.toRadians(90+180))
            .setReversed(true)
            .splineTo(new Vector2d(-12, -20), Math.toRadians(135-90))
            .waitSeconds(3)
            .splineTo(new Vector2d(36, -45), Math.toRadians(90+180))
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
    static public RoadRunnerBotEntity red2rout(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
           // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
           .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, 50,Math.toRadians(126) ))
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(135+180))
                .waitSeconds(3)
                .splineTo(new Vector2d(-12, 30), Math.toRadians(90))
                .splineTo(new Vector2d(-12, 48), Math.toRadians(90))
                .strafeTo(new Vector2d(2, 48))
                .setReversed(true)
                .splineTo(new Vector2d(10, 24), Math.toRadians(135+180))
                .setReversed(false)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(135))
                .waitSeconds(3)
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
                .splineTo(new Vector2d(12, 45), Math.toRadians(90))
                .splineTo(new Vector2d(-24, 24), Math.toRadians(135))
                .waitSeconds(3)



      .build());


        return myBot;

    }
    static public RoadRunnerBotEntity red1rout(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60,  24, Math.toRadians(180) ))
            .waitSeconds(6)
            .splineTo(new Vector2d(0,  10), Math.toRadians(135))
            .waitSeconds(3)
            .strafeTo(new Vector2d(36, 10))
            .waitSeconds(0.1)
            .splineTo(new Vector2d(36,  45), Math.toRadians(90))
            .waitSeconds(3)
            .setReversed(true)
            .splineTo(new Vector2d(36,  10), Math.toRadians(135+180))
            .setReversed(false)
            .splineTo(new Vector2d(0,  10), Math.toRadians(135))
            .waitSeconds(3)

            .build());


        return myBot;

    }
}