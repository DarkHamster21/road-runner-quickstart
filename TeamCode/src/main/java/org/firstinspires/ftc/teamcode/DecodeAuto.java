package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "DECODE 9 Artifact Auto RR (Red Side)", group = "Competition")
public class DecodeAuto extends LinearOpMode {

    // Hardware declarations (Ensure these names match your configuration)
    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private Servo gateServo; // Used for dropping/shooting artifacts

    // Mechanism Constants
    private static final double SHOOTER_POWER = 0.85; // Max speed for outtake mechanism
    private static final double INTAKE_POWER = 0.9;   // High power for quick intake

    // Servo Positions
    private static final double GATE_OPEN = 0.8;      // Position to release an artifact
    private static final double GATE_CLOSED = 0.2;    // Position to hold artifacts

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "RightOuttakeMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "LeftOuttakeMotor");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        // === FIELD COORDINATE SETUP (Red Alliance, staying in X < 0) ===

        // Starting pose: Red Alliance, Tile D1 (Approx -60, -36), facing the field
        Pose2d beginPose = new Pose2d(-60, -36, Math.toRadians(0));

        // Initialize MecanumDrive with starting pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Scoring Position: A spot on the Red side facing the central scoring goal/backdrop
        // The robot stays close to the wall but angles toward the scoring area.
        Pose2d scoringPose = new Pose2d(-40, -40, Math.toRadians(135));

        // Artifact Stacks (Two accessible stacks/piles on the Red side, sweeping along the Y-axis)

        // Stack 1: Closest to the center line (most dangerous, grab first)
        Pose2d artifactStack1 = new Pose2d(-12, -40, Math.toRadians(0));

        // Stack 2: Further back from the center, still along the same line (same Y, different X)
        // This simulates sweeping back towards the alliance side for the second pile.
        Pose2d artifactStack2 = new Pose2d(-28, -40, Math.toRadians(0));

        // Parking Position: Park near the scoring goal/wall for max points
        Pose2d parkPose = new Pose2d(-60, -12, Math.toRadians(90));

        // Ensure the gate is closed at the start to hold preloaded artifacts
        gateServo.setPosition(GATE_CLOSED);


        // === ROAD RUNNER TRAJECTORY ACTIONS ===

        // 1. Leave start and move to scoring position
        Action trajLeaveAndScore = drive.actionBuilder(beginPose)
                .lineToX(-48) // Move off the wall
                .splineToLinearHeading(scoringPose, Math.toRadians(45)) // Curve to the scoring angle
                .build();

        // 2. Go to the first artifact stack (from scoringPose)
        // This uses a lineToX to quickly move to the collection spot
        Action trajToIntake1 = drive.actionBuilder(scoringPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(artifactStack1, Math.toRadians(0))
                .build();

        // 3. Return to scoring position from the first stack
        Action trajScoreFromIntake1 = drive.actionBuilder(artifactStack1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(scoringPose, Math.toRadians(225))
                .build();

        // 4. Go to the second artifact stack
        // The robot moves from the scoring position directly back to the second stack location
        Action trajToIntake2 = drive.actionBuilder(scoringPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(artifactStack2, Math.toRadians(0))
                .build();

        // 5. Return to scoring position from the second stack
        Action trajScoreFromIntake2 = drive.actionBuilder(artifactStack2)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(scoringPose, Math.toRadians(225))
                .build();

        // 6. Park in the base zone (from scoringPose)
        Action trajPark = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();


        // === CUSTOM MECHANISM ACTIONS ===
        // (These actions remain the same)

        Action spinUpShooter = telemetryPacket -> {
            shooterMotor1.setPower(SHOOTER_POWER);
            shooterMotor2.setPower(SHOOTER_POWER);
            sleep(1000); // Wait for shooter to reach speed
            return false; // Action complete
        };

        Action shootThreeArtifacts = telemetryPacket -> {
            for (int i = 0; i < 3; i++) {
                // Open gate to release one artifact
                gateServo.setPosition(GATE_OPEN);
                sleep(200); // Dwell time for artifact to pass

                // Close gate to prepare for next artifact
                gateServo.setPosition(GATE_CLOSED);
                sleep(300); // Time for mechanism to reset
            }
            return false; // Action complete
        };

        Action stopShooter = telemetryPacket -> {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
            return false; // Action complete
        };

        Action intakeThreeArtifacts = telemetryPacket -> {
            intakeMotor.setPower(INTAKE_POWER);
            sleep(2500); // Time to intake 3 artifacts
            intakeMotor.setPower(0);
            return false; // Action complete
        };

        telemetry.addData("Status", "Initialized - DECODE 9 Artifact Auto (Red Side)");
        telemetry.addData("Starting Pose", beginPose.toString());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // === AUTONOMOUS SEQUENCE (9 ARTIFACTS - Two Field Intakes) ===
        Actions.runBlocking(
                new SequentialAction(
                        // 1. Score 3 pre-loaded artifacts
                        trajLeaveAndScore,
                        spinUpShooter,
                        shootThreeArtifacts,
                        stopShooter,

                        // 2. Go to 1st stack, intake, and score (3 more)
                        trajToIntake1,
                        intakeThreeArtifacts,
                        trajScoreFromIntake1,
                        spinUpShooter,
                        shootThreeArtifacts,
                        stopShooter,

                        // 3. Go to 2nd stack, intake, and score (3 more)
                        trajToIntake2,
                        intakeThreeArtifacts,
                        trajScoreFromIntake2,
                        spinUpShooter,
                        shootThreeArtifacts,
                        stopShooter,

                        // 4. Park
                        trajPark
                )
        );

        telemetry.addData("Status", "Auto Complete!");
        telemetry.addData("Total Artifacts Scored", "9 (Max on Red Side)");
        telemetry.update();

        sleep(1000);
    }
}