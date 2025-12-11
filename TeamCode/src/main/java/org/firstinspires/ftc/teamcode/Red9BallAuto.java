package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "9 Ball RED auto", group = "Competition")
public class Red9BallAuto extends LinearOpMode {

    // Hardware declarations
    private DcMotorEx IntakeMotor;
    private DcMotorEx LeftOuttakeMotor;
    private DcMotorEx RightOuttakeMotor;
    private DcMotorEx SpindexerMotor;
    private Servo IntakeServo;
    private Servo OutakeServo;

    // Mechanism Constants
    private static final double INTAKE_MOTOR_VELOCITY = 2787.9;
    private static final double INTAKE_START_POSITION = 0.555;
    private static final double INTAKE_END_POSITION = 0.42;
    final double OUTTAKE_SERVO_START_POSITION = 0;
    final double OUTTAKE_SERVO_SHOOT_POSITION = 0.35;
    final double SpindexerEncoderPulsesPerRevolution = (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * 28;
    double spindexerRotation = 0;
    final double MaxOuttakeVelocity = 7.25;
    double OuttakeVelocity = MaxOuttakeVelocity * 1;
    final double OuttakeMotorPulsesPerRevolution = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * 28.0);

    int spindexerState = 0;
    long spindexerWaitTime = 0;

    long IntakeWaitTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        LeftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "RightOuttakeMotor");
        RightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "LeftOuttakeMotor");
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "SpindexerMotor");
        RightOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        OutakeServo = hardwareMap.get(Servo.class,"OuttakeServo");

        // === FIELD COORDINATE SETUP ===
        Pose2d beginPose = new Pose2d(-60, 36, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d scoringPose = new Pose2d(-30, 30, Math.toRadians(310));

        // Stack 1 Positions
        Pose2d approachArtifactStack1 = new Pose2d(-12, 32, Math.toRadians(90));
        Pose2d endOfArtifactStack1 = new Pose2d(-12, 48, Math.toRadians(90));

        // Stack 2 Positions
        Pose2d approachArtifactStack2 = new Pose2d(12, 32, Math.toRadians(90));
        Pose2d endOfArtifactStack2 = new Pose2d(12, 48, Math.toRadians(90));

        Pose2d parkPose = new Pose2d(36, 32, Math.toRadians(90));

        // Initial mechanism setup
        IntakeServo.setPosition(INTAKE_START_POSITION);
        OutakeServo.setPosition(OUTTAKE_SERVO_START_POSITION);
        SpindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SpindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // === CUSTOM MECHANISM ACTIONS ===
        Action spinUpShooter = telemetryPacket -> {
            RightOuttakeMotor.setVelocity(OuttakeMotorPulsesPerRevolution * OuttakeVelocity);
            LeftOuttakeMotor.setVelocity(OuttakeMotorPulsesPerRevolution * OuttakeVelocity);
            sleep(100);
            return false;
        };

        Action shootThreeArtifacts = telemetryPacket -> {
            for (int i = 0; i < 3; i++) {
                IntakeServo.setPosition(INTAKE_END_POSITION);
                sleep(200);
                IntakeServo.setPosition(INTAKE_START_POSITION);
                sleep(100);
                spindexerRotation += SpindexerEncoderPulsesPerRevolution/3;
                SpindexerMotor.setTargetPosition((int) spindexerRotation);
                SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SpindexerMotor.setPower(0.8);
                sleep(400);
                OutakeServo.setPosition(OUTTAKE_SERVO_SHOOT_POSITION);
                sleep(200);
                OutakeServo.setPosition(OUTTAKE_SERVO_START_POSITION);
            }
            return false;
        };

        Action stopShooter = telemetryPacket -> {
            LeftOuttakeMotor.setVelocity(0);
            RightOuttakeMotor.setVelocity(0);
            return false;
        };

        Action startIntakeMotor = telemetryPacket -> {
            IntakeMotor.setVelocity(INTAKE_MOTOR_VELOCITY);
            return false; // This action is instantaneous
        };

        Action stopIntakeMotor = telemetryPacket -> {
            IntakeWaitTime = System.currentTimeMillis() + 300;
            IntakeMotor.setVelocity(0);
            return false;
        };



        // ... (other actions are unchanged) ...

        Action runSpindexerForIntake = telemetryPacket -> {
            switch (spindexerState) {
                case 0: // Initial delay state
                    // timer for how long to wait before the first rotation.
                    spindexerWaitTime = System.currentTimeMillis() + 300;
                    spindexerState = 1; // Move to the "wait for delay" state
                    break;

                case 1: // Wait for the initial delay to finish
                    if (System.currentTimeMillis() >= spindexerWaitTime) {
                        // Timer is up, now start the first rotation
                        spindexerRotation += SpindexerEncoderPulsesPerRevolution / 3;
                        SpindexerMotor.setTargetPosition((int) spindexerRotation);
                        SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        SpindexerMotor.setPower(0.8);
                        spindexerState = 2; // Move to the "wait for move to finish" state
                    }
                    break;

                case 2: // Wait for the first rotation to complete
                    if (!SpindexerMotor.isBusy()) {
                        spindexerWaitTime = System.currentTimeMillis() + 400; // Set a timer for the pause
                        SpindexerMotor.setPower(0); // Stop the motor
                        spindexerState = 3; // Move to the "pause" state
                    }
                    break;

                case 3: // Wait for the pause to finish
                    if (System.currentTimeMillis() >= spindexerWaitTime) {
                        // Start the second rotation
                        spindexerRotation += SpindexerEncoderPulsesPerRevolution / 3;
                        SpindexerMotor.setTargetPosition((int) spindexerRotation);
                        SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        SpindexerMotor.setPower(0.8);
                        spindexerState = 4; // Move to the "wait for second move" state
                    }
                    break;

                case 4: // Wait for the second rotation to complete
                    if (!SpindexerMotor.isBusy()) {
                        // Intake of 3 balls should be complete.
                        // We are now finished.
                        SpindexerMotor.setPower(0);
                        spindexerState = 5; // Move to the final "finished" state
                    }
                    break;
                case 5: // Initial delay state
                    // timer for how long to wait before the first rotation.
                    spindexerWaitTime = System.currentTimeMillis() + 200;
                    spindexerState = 6; // Move to the "wait for delay" state
                    break;

                case 6: // The action is complete
                    spindexerState = 0; // Reset for the next time it's called
                    return false; // Signal that the action is finished
            }

            return true; // Signal that the action is still running
        };

        //  ROAD RUNNER MOVEMENT ACTIONS

        Action trajLeaveAndScore = drive.actionBuilder(beginPose)
                .lineToX(-48)
                .splineToLinearHeading(scoringPose, Math.toRadians(215))
                .build();

        // **MODIFIED**: Single, continuous action to drive TO and THROUGH the first stack
        Action driveAndCollect1 = drive.actionBuilder(scoringPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(approachArtifactStack1, Math.toRadians(90))
                // Use afterDisp(0, ...) to run an action at the start of the *next* segment.
                // This starts the intake right as the robot moves forward to collect.
                .afterDisp(0, new ParallelAction(startIntakeMotor, runSpindexerForIntake))
                .splineToLinearHeading(endOfArtifactStack1, Math.toRadians(90))
                // Use stopAndAdd() to run an instantaneous action at the end of the trajectory.
                .stopAndAdd(stopIntakeMotor)
                .build();
        // **MODIFIED**: Return trajectory now starts from the correct end position
        Action trajScoreFromIntake1 = drive.actionBuilder(endOfArtifactStack1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(scoringPose, Math.toRadians(135))
                .build();

        // **MODIFIED**: Single action for the second stack
        Action driveAndCollect2 = drive.actionBuilder(scoringPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(approachArtifactStack2, Math.toRadians(90))
                // Use afterDisp(0, ...) to run an action at the start of the *next* segment.
                // This starts the intake right as the robot moves forward to collect.
                .afterDisp(0, new ParallelAction(startIntakeMotor, runSpindexerForIntake))
                .splineToLinearHeading(endOfArtifactStack2, Math.toRadians(90))
                // Use stopAndAdd() to run an instantaneous action at the end of the trajectory.
                .stopAndAdd(stopIntakeMotor)
                .build();


        // Return trajectory for the second line of Artifacts
        Action trajScoreFromIntake2 = drive.actionBuilder(endOfArtifactStack2)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(scoringPose, Math.toRadians(90))
                .build();

        Action trajPark = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Pose", beginPose.toString());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // === AUTONOMOUS SEQUENCE ===
        Actions.runBlocking(
                new SequentialAction(
                        // 1. Score 3 pre-loaded artifacts
                        trajLeaveAndScore,
                        //spinUpShooter,
                        shootThreeArtifacts,
                        stopShooter,

                        // 2. Drive to, collect from, and score from the 1st stack
                        driveAndCollect1, // This single action now handles the entire collection process
                        new ParallelAction( // Optimize by spinning up shooter while returning
                                trajScoreFromIntake1
                                //spinUpShooter
                        ),
                        shootThreeArtifacts,
                        stopShooter,

                        // 3. Drive to, collect from, and score from the 2nd stack
                        driveAndCollect2, // Same pattern for the second stack
                        new ParallelAction(
                                trajScoreFromIntake2
                                //spinUpShooter
                        ),
                        shootThreeArtifacts,
                        stopShooter,

                        // 4. Park
                        trajPark
                )
        );

        telemetry.addData("Status", "Auto Complete!");
        telemetry.update();
        sleep(1000);
    }
}
