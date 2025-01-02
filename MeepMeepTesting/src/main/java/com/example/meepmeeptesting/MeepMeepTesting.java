package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(18, -60, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)

                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(9, -33, Math.toRadians(90)), -11)

                        .setTangent(-11)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(38, -29), -11)
                        .splineToConstantHeading(new Vector2d(38, -11), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(42, -11), -1)
                        .splineToConstantHeading(new Vector2d(42, -55), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(45, -11), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(58, -11), -11)
                        .splineToConstantHeading(new Vector2d(58, -55), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(58, -11), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(62, -11), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(62, -55), Math.PI / 2)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(270)), Math.PI / 2)
                        .strafeTo(new Vector2d(40, -55))

                        .splineToLinearHeading(new Pose2d(7, -33, Math.toRadians(90)), Math.PI / 2)



                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}