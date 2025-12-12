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

@Autonomous(name = "9 Ball BLUE auto", group = "Competition")
public class Blue9BallAuto extends LinearOpMode {

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
    final double OuttakeVelocityControl = 0.8;
    double OuttakeVelocity = MaxOuttakeVelocity * OuttakeVelocityControl;
    final double OuttakeMotorPulsesPerRevolution = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * 28.0);

     int spindexerState = 0;
     long spindexerWaitTime = 0;


//     Shooting states
    int shootState = 0;
    long shootWaitTime = 0;
    int shootCount = 0;

     long pickupWaitTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        LeftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "LeftOuttakeMotor");
        RightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "RightOuttakeMotor");
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "SpindexerMotor");
        LeftOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        OutakeServo = hardwareMap.get(Servo.class,"OuttakeServo");

        // === FIELD COORDINATE SETUP ===
        Pose2d beginPose = new Pose2d(-60, -36, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d scoringPose = new Pose2d(-30, -30, Math.toRadians(50));

        // Stack 1 Positions
        Pose2d approachArtifactStack1 = new Pose2d(-12, -32, Math.toRadians(270));
        Pose2d endOfArtifactStack1 = new Pose2d(-12, -48, Math.toRadians(270));

        // Stack 2 Positions
        Pose2d approachArtifactStack2 = new Pose2d(12, -32, Math.toRadians(270));
        Pose2d endOfArtifactStack2 = new Pose2d(12, -48, Math.toRadians(270));

        Pose2d parkPose = new Pose2d(36, -32, Math.toRadians(90));

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
        // ... inside runOpMode() ...
        Action shootThreeArtifacts = telemetryPacket -> {
            switch (shootState) {
                case 0: // Start first shot - move servo
                    IntakeServo.setPosition(INTAKE_END_POSITION);
                    shootWaitTime = System.currentTimeMillis() + 200;
                    shootState = 1;
                    break;
                case 1: // Retract servo
                    if (System.currentTimeMillis() >= shootWaitTime) {
                        IntakeServo.setPosition(INTAKE_START_POSITION);
                        shootWaitTime = System.currentTimeMillis() + 100;
                        shootState = 2;
                    }
                    break;
                case 2: // Move spindexer
                    if (System.currentTimeMillis() >= shootWaitTime) {
                        spindexerRotation += SpindexerEncoderPulsesPerRevolution / 3;
                        SpindexerMotor.setTargetPosition((int) spindexerRotation);
                        SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        SpindexerMotor.setPower(0.8);
                        shootWaitTime = System.currentTimeMillis() + 400; // Wait for spindexer
                        shootState = 3;
                    }
                    break;
                case 3: // Fire the ring
                    if (System.currentTimeMillis() >= shootWaitTime && !SpindexerMotor.isBusy()) {
                        OutakeServo.setPosition(OUTTAKE_SERVO_SHOOT_POSITION);
                        shootWaitTime = System.currentTimeMillis() + 200;
                        shootState = 4;
                    }
                    break;
                case 4: // Retract outtake servo
                    if (System.currentTimeMillis() >= shootWaitTime) {
                        OutakeServo.setPosition(OUTTAKE_SERVO_START_POSITION);
                        shootCount++;
                        if (shootCount >= 3) {
                            // Finished all three shots
                            shootState = 0; // Reset for next time
                            shootCount = 0;
                            return false; // Action is complete
                        } else {
                            // Go back to start the next shot
                            shootState = 0;
                        }
                    }
                    break;
            }
            return true; // Action is still running
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
            IntakeMotor.setVelocity(0);
            return false;
        };


//TODO: slow intake motor when spindexer moves
        Action runSpindexerForIntake = telemetryPacket -> {
            switch (spindexerState) {
                case 0: // Initial delay state
                    // timer for how long to wait before the first rotation.
                    spindexerWaitTime = System.currentTimeMillis() + 300;
                    spindexerState = 1; // Move to the "wait for delay" state
                    break;
                case 1: // Start the first rotation
                    spindexerWaitTime = System.currentTimeMillis() + 300;
                    spindexerRotation += SpindexerEncoderPulsesPerRevolution / 3;
                    SpindexerMotor.setTargetPosition((int) spindexerRotation);
                    SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SpindexerMotor.setPower(0.8);
                    spindexerState = 2; // Move to the "wait for move to finish" state
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
                        spindexerWaitTime = System.currentTimeMillis() + 300;
                        SpindexerMotor.setPower(0);
                        spindexerState = 5; // Move to the "finished" state
                    }
                    break;


                case 5: // The action is complete
                    spindexerState = 0; // Reset for the next time it's called
                    return false; // Signal that the action is finished
            }

            return true; // Signal that the action is still running
        };




        // === ROAD RUNNER TRAJECTORY ACTIONS ===

        Action trajLeaveAndScore = drive.actionBuilder(beginPose)
                .lineToX(-48)
                .splineToLinearHeading(scoringPose, Math.toRadians(45))
                .build();

        // **MODIFIED**: Single, continuous action to drive TO and THROUGH the first stack
        Action DriveToArtifact1 = drive.actionBuilder(scoringPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(approachArtifactStack1, Math.toRadians(270))
                .build();

        Action DriveThroughArtifact1 = drive.actionBuilder(approachArtifactStack1)
                .splineToLinearHeading(endOfArtifactStack1, Math.toRadians(270))
                .build();

        // **MODIFIED**: Return trajectory now starts from the correct end position
        Action trajScoreFromIntake1 = drive.actionBuilder(endOfArtifactStack1)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(scoringPose, Math.toRadians(180))
                .build();

        // **MODIFIED**: Single action for the second stack
        Action DriveToArtifact2 = drive.actionBuilder(scoringPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(approachArtifactStack2, Math.toRadians(270))

                .build();
        Action DriveThroughArtifact2 = drive.actionBuilder(approachArtifactStack2)
                .splineToLinearHeading(endOfArtifactStack2, Math.toRadians(270))
                .build();




        // **MODIFIED**: Return trajectory for the second stack
        Action trajScoreFromIntake2 = drive.actionBuilder(endOfArtifactStack2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(scoringPose, Math.toRadians(180))
                .build();

        Action trajPark = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Pose", beginPose.toString());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //AUTONOMOUS SEQUENCE
        Actions.runBlocking(
                new SequentialAction(
                        // 1. Score 3 pre-loaded artifacts
                        trajLeaveAndScore,
                        spinUpShooter,
                        shootThreeArtifacts,
                        stopShooter,
                        startIntakeMotor,

                        // 2. Drive to, collect from, and score from the 1st stack
                        DriveToArtifact1, // This single action now handles the entire collection process
                        new ParallelAction(
                                DriveThroughArtifact1,
                                runSpindexerForIntake
                        ),
                        new ParallelAction(
                                trajScoreFromIntake1,
                                stopIntakeMotor,
                                spinUpShooter
                        ),

                        shootThreeArtifacts,
                        stopShooter,
                        startIntakeMotor,

                        // 3. Drive to, collect from, and score from the 2nd stack
                        DriveToArtifact2, // Same pattern for the second stack
                        new ParallelAction(
                                DriveThroughArtifact2,
                                runSpindexerForIntake
                        ),

                        new ParallelAction(
                                trajScoreFromIntake2,
                                stopIntakeMotor,
                                spinUpShooter
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
