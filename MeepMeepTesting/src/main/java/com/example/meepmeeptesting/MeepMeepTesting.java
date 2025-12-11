package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Define poses from your latest DecodeAuto file
        Pose2d beginPose = new Pose2d(63, 36, Math.toRadians(0));
        Pose2d scoringPose = new Pose2d(40, 40, Math.toRadians(50));

        // --- NOTE: The order of visiting these is now different ---
        Pose2d artifactStack1 = new Pose2d(-12, -40, Math.toRadians(0)); // Furthest stack
        Pose2d artifactStack2 = new Pose2d(-28, -40, Math.toRadians(0)); // Closest stack

        Pose2d parkPose = new Pose2d(-60, -12, Math.toRadians(90));

        // Bot for the "DECODE 9 Artifact Auto RR (Red Side)"
        RoadRunnerBotEntity autoBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Build the full autonomous sequence with the UPDATED ORDER
        autoBot.runAction(autoBot.getDrive().actionBuilder(beginPose)
                // 1. Leave start and move to scoring position
                .lineToX(-48)
                .splineToLinearHeading(scoringPose, Math.toRadians(45))
                .waitSeconds(1.5) // Simulate shooting preloads

                // 2. Go to CLOSEST stack (Stack 2), intake, and return to score
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(artifactStack2, Math.toRadians(0)) // Path to closest stack
                .waitSeconds(2.5) // Simulate intaking
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(scoringPose, Math.toRadians(225))
                .waitSeconds(1.5) // Simulate shooting

                // 3. Go to FURTHEST stack (Stack 1), intake, and return to score
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(artifactStack1, Math.toRadians(0)) // Path to furthest stack
                .waitSeconds(2.5) // Simulate intaking
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(scoringPose, Math.toRadians(225))
                .waitSeconds(1.5) // Simulate shooting

                // 4. Park
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build()
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(autoBot)   // Add the auto path
                .start();
    }
}
