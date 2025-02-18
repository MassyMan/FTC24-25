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
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .strafeTo(new Vector2d(10, -32))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(36, -40, Math.toRadians(270)), Math.PI / 2)
                                .strafeTo(new Vector2d(36, -8))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(50, -8, Math.toRadians(270)), Math.PI / 2)
                                .strafeTo(new Vector2d(50, -50))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(48, -8, Math.toRadians(270)), Math.PI / 2)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(60, -8, Math.toRadians(270)), Math.PI / 2)
                                .strafeTo(new Vector2d(60, -50))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
