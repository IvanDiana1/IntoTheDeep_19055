package com.example.mipmiptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MipMipTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 35, Math.toRadians(180), Math.toRadians(180), 9.75)
               // .setStartPose(new Pose2d(-8.5,63,Math.toRadians(-90)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                        .splineToConstantHeading(new Vector2d(16,1),Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(23, -31, Math.toRadians(-30)))
                        .splineToLinearHeading(new Pose2d(19, -41, Math.toRadians(-120)),Math.toRadians(0))

                        .lineToLinearHeading(new Pose2d(24, -42.15, Math.toRadians(-20)))
                        .splineToLinearHeading(new Pose2d(19.6, -48, Math.toRadians(-120)),Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(35, -46, Math.toRadians(-90)))

                        .splineToLinearHeading(new Pose2d(14, -31.5, Math.toRadians(-178)),Math.toRadians(20))
                        .lineToLinearHeading(new Pose2d(18.5,-4,  Math.toRadians(-10)))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(13,-32,Math.toRadians(180)),Math.toRadians(20))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}