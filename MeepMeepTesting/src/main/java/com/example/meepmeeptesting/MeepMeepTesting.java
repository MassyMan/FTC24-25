package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(10, -60, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)

                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(10, -32, Math.toRadians(90)), Math.PI / 2) // Bar position
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(35, -43, Math.toRadians(90)), Math.PI / 2)
                        .splineToLinearHeading(new Pose2d(35, -15, Math.toRadians(90)), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(45, -11), 11) // First block position (in front of it, ready to push it)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(45, -41), Math.PI / 2) // Pushed against wall
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(45, -15), Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(57, -11), 11) // Second block position (in front of it, ready to push it)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(57, -41), Math.PI / 2) // Pushed against wall
                        .splineToConstantHeading(new Vector2d(57, -11), Math.PI / 2) // WALL POSITION
                        .splineToConstantHeading(new Vector2d(61, -11), 11) // Third block position (in front of it, ready to push it)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(61, -41), Math.PI / 2) // Pushed against wall
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(50, -50, Math.toRadians(270)), 11) // WALL POSITION
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}