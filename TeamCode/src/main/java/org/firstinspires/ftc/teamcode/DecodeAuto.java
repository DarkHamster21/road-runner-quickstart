package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "DECODE Auto RR", group = "Competition")
public class DecodeAuto extends LinearOpMode {

    // Hardware declarations
    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private Servo gateServo;

    // Shooter constants (adjust for your robot)
    private static final double SHOOTER_POWER = 0.85;

    // Gate positions
    private static final double GATE_OPEN = 0.8;
    private static final double GATE_CLOSED = 0.2;

    // Intake power
    private static final double INTAKE_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        //gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Starting pose - Red alliance, left side
        // Adjust based on your starting tile position
        Pose2d beginPose = new Pose2d(-60, -36, Math.toRadians(0));

        // Initialize MecanumDrive with starting pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Build action sequences

        // 1. Leave starting zone and move to shooting position
        TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                .lineToX(-48) // Leave starting zone (3 points)
                .strafeToLinearHeading(new Vector2d(-36, -24), Math.toRadians(45)); // Move to shooting position

        // 2. Optional: Get more artifacts after scoring
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-36, -24, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(0)); // Go to artifact pickup

        // 3. Return to shooting position
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-48, -48, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-36, -24), Math.toRadians(45)); // Back to shoot

        // 4. Park in base zone
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-36, -24, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-60, -48), Math.toRadians(0)); // Park in base

        // Create custom actions for game mechanisms
//        Action closeGate = telemetryPacket -> {
//            gateServo.setPosition(GATE_CLOSED);
//            return false;
//        };

        Action spinUpShooter = telemetryPacket -> {
            shooterMotor1.setPower(SHOOTER_POWER);
            shooterMotor2.setPower(SHOOTER_POWER);
            sleep(1000); // Let shooter reach speed
            return false;
        };

        Action shootThreeArtifacts = telemetryPacket -> {
            for (int i = 0; i < 3; i++) {
                intakeMotor.setPower(INTAKE_POWER);
                sleep(500);
                intakeMotor.setPower(0);
                sleep(300);
            }
            return false;
        };

        Action stopShooter = telemetryPacket -> {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
            return false;
        };

        Action intakeArtifacts = telemetryPacket -> {
            intakeMotor.setPower(INTAKE_POWER);
            sleep(2000);
            intakeMotor.setPower(0);
            return false;
        };

        telemetry.addData("Status", "Initialized - DECODE Auto with Road Runner");
        telemetry.addData("Starting Pose", beginPose.toString());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // === AUTONOMOUS SEQUENCE ===

        // Simple reliable auto: Leave + Score preloads + Park
        Actions.runBlocking(
                new SequentialAction(
                        //closeGate,
                        tab1.build(), // Leave zone and move to shooting position
                        spinUpShooter,
                        shootThreeArtifacts, // Score 3 preloaded artifacts
                        stopShooter,
                        tab4.build() // Park in base zone
                )
        );

        /* ADVANCED AUTO - Uncomment if you want to pick up more artifacts
        Actions.runBlocking(
            new SequentialAction(
                closeGate,
                tab1.build(), // Leave + move to shooting position
                spinUpShooter,
                shootThreeArtifacts, // Score preloads
                stopShooter,
                tab2.build(), // Go get more artifacts
                intakeArtifacts,
                tab3.build(), // Return to shooting position
                spinUpShooter,
                shootThreeArtifacts, // Score additional artifacts
                stopShooter,
                tab4.build() // Park
            )
        );
        */

        telemetry.addData("Status", "Auto Complete!");
        telemetry.addData("Estimated Points", "Leave(3) + Classified(9) + Park(5+) = 17+");
        telemetry.update();

        sleep(1000);
    }
}