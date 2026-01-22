package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -15, Math.toRadians(180)))
                //TURN ON SHOOTER
                .lineToX(-20)
                .turn(Math.toRadians(50))
                .waitSeconds(2)//SHOOT BALLS
                .strafeToConstantHeading(new Vector2d(35,-15))
                //.lineToX(-10)
                .turn(Math.toRadians(40))
                .lineToY(-50)
                .waitSeconds(0.5)//INTAKE BALLS
                .lineToY(-26)
                .setTangent(40)
                .splineTo(new Vector2d(-20, -15), Math.PI / 2)
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToLinearHeading(new Vector2d(5,-15), Math.toRadians(150))
                .build());
        //x; -20 y; -15 for shooting

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
        /* FOR BLUE SIDE PPG:
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -15, Math.toRadians(180)))
                //TURN ON SHOOTER
                .lineToX(-35)
                .turn(Math.toRadians(40))
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToConstantHeading(new Vector2d(-3,-15))
                //.lineToX(-10)
                .turn(Math.toRadians(-130))
                .lineToY(-45)
                .waitSeconds(0.2)//INTAKE BALLS
                .setTangent(150)
                .splineTo(new Vector2d(-35, -15), Math.PI / 2)
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToLinearHeading(new Vector2d(5,-15), Math.toRadians(150))
                .build());
    */
        /* FOR BLUE SIDE PGP:
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -15, Math.toRadians(180)))
                //TURN ON SHOOTER
                .lineToX(-20)
                .turn(Math.toRadians(50))
                .waitSeconds(2)//SHOOT BALLS
                .strafeToConstantHeading(new Vector2d(13,-15))
                //.lineToX(-10)
                .turn(Math.toRadians(40))
                .lineToY(-50)
                .waitSeconds(0.5)//INTAKE BALLS
                .lineToY(-30)
                .setTangent(40)
                .splineTo(new Vector2d(-20, -15), Math.PI / 2)
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToLinearHeading(new Vector2d(5,-15), Math.toRadians(150))
                .build());
         */
        /* FOR BLUE SIDE GPP:
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -15, Math.toRadians(180)))
                //TURN ON SHOOTER
                .lineToX(-20)
                .turn(Math.toRadians(50))
                .waitSeconds(2)//SHOOT BALLS
                .strafeToConstantHeading(new Vector2d(35,-15))
                //.lineToX(-10)
                .turn(Math.toRadians(40))
                .lineToY(-50)
                .waitSeconds(0.5)//INTAKE BALLS
                .lineToY(-26)
                .setTangent(40)
                .splineTo(new Vector2d(-20, -15), Math.PI / 2)
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToLinearHeading(new Vector2d(5,-15), Math.toRadians(150))
                .build());
         */
        /* FOR RED SIDE PPG:
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, 15, Math.toRadians(180)))
                //TURN ON SHOOTER
                .lineToX(-20)
                .turn(Math.toRadians(-50))
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToConstantHeading(new Vector2d(-10,15))
                //.lineToX(-10)
                .turn(Math.toRadians(-40))
                .lineToY(50)//INTAKE ON
                .waitSeconds(0.2)//INTAKE BALLS
                .setTangent(Math.toRadians(-135))
                .splineTo(new Vector2d(-20, 15), Math.PI / -2)
                //MOVE BALLS UP
                .waitSeconds(2)//SHOOT BALLS
                .strafeToLinearHeading(new Vector2d(5,15), Math.toRadians(40))
                .build());
  */
    /*
    MOVING BALLS UP WHILE SHOOTING:
                //shoot topLauncher.moveUp(0.8, 2),
                        bottomLauncher.moveUp(0.8,1),
                //shoot topLauncher.moveUp(0.8, 2),
                        intake.takein(),
                        bottomLauncher.moveUp(0.8,2),
                //shoot topLauncher.moveUp(0.8,2),
     */

}
