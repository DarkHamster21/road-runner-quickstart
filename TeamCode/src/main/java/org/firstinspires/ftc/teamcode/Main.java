package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Main", group = "Competition")
public class Main extends LinearOpMode {

    // Controller 1 Stuff -----------------------------------
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;

    public void Controller1Init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BackRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Controller1Loop() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // Controller 2 stuff -----------------------
    DcMotorEx LeftOuttakeMotor;
    DcMotorEx RightOuttakeMotor;
    DcMotorEx IntakeMotor;
    DcMotorEx SpindexerMotor;
    Servo IntakeServo;
    Servo OuttakeServo;

    // intake servo stuff
    double intakeStartTime = 0;
    boolean intakeDepositing = false;
    final double intakeDepositTime = 1.0;

    // outtake servo stuff
    double outtakeStartTime = 0;
    boolean outtakeShooting = false;
    final double outtakeShootTime = 1.0;

    // Intake Servo Angles
    final double IntakeServoDefaultAngle = 0.555;
    final double IntakeServoDepositeAngle = 0.42;

    // Outtake Servo Angles
    final double OuttakeServoDefaultAngle = 0;
    final double OuttakeServoShootAngle = 0.2;

    final double SpindexerEncoderPulsesPerRevolution = (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * 28;
    double spinDexerRotation = 0;

    boolean IntakeToggle = false;
    boolean PreviousIntakeGamepad;
    boolean previousGamepadY = false;

    boolean PreviousGamepadOuttake = false;
    boolean OuttakeEnabled = false;

    final double MaxOuttakeVelocity = 7.25; // rotations per seconds
    double OuttakeVelocity = MaxOuttakeVelocity * 0.75; // in rotations per second
    final double OuttakeMotorPulsesPerRevolution = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * 28.0);


    public void Controller2Init() {
        LeftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "LeftOuttakeMotor");
        RightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "RightOuttakeMotor");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "SpindexerMotor");

        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        OuttakeServo = hardwareMap.get(Servo.class, "OuttakeServo");

        LeftOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SpindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SpindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IntakeServo.setPosition(IntakeServoDefaultAngle);
        OuttakeServo.setPosition(OuttakeServoDefaultAngle);
    }

    public void Controller2Loop() {

        // Outtake toggling
        if (gamepad2.a && !PreviousGamepadOuttake) {
            OuttakeEnabled = !OuttakeEnabled;
        }
        PreviousGamepadOuttake = gamepad2.a;

        if(OuttakeEnabled) {
            RightOuttakeMotor.setVelocity(OuttakeMotorPulsesPerRevolution * OuttakeVelocity);
            LeftOuttakeMotor.setVelocity(OuttakeMotorPulsesPerRevolution * OuttakeVelocity);
        } else {
            RightOuttakeMotor.setVelocity(0);
            LeftOuttakeMotor.setVelocity(0);
        }

        // intake related
        if (gamepad2.x && !PreviousIntakeGamepad) {
            IntakeToggle = !IntakeToggle;
        }
        PreviousIntakeGamepad = gamepad2.x;

        if (IntakeToggle){
            IntakeMotor.setPower(1); // On
        } else {
            IntakeMotor.setPower(0); // Off
        }

        if (gamepad2.y && !previousGamepadY){
            // rotates by 1/3 of a total revolution
            spinDexerRotation += SpindexerEncoderPulsesPerRevolution/3;

            SpindexerMotor.setTargetPosition((int) spinDexerRotation);
            SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SpindexerMotor.setPower(0.3);
        }

        previousGamepadY = gamepad2.y;

        // Trigger intake deposit
        if (gamepad2.dpad_up && !intakeDepositing ) { // && (outtakeStartTime + outtakeShootTime) <= getRuntime()
            intakeDepositing = true;
            intakeStartTime = getRuntime();
            IntakeServo.setPosition(IntakeServoDepositeAngle);
        }

        // Trigger outtake shoot
        if (gamepad2.dpad_down && !outtakeShooting) {
            outtakeShooting = true;
            outtakeStartTime = getRuntime();
            OuttakeServo.setPosition(OuttakeServoShootAngle);
        }

        // Intake timing reset
        if (intakeDepositing && getRuntime() - intakeStartTime >= intakeDepositTime) {
            intakeDepositing = false;
            IntakeServo.setPosition(IntakeServoDefaultAngle);
        }

        // Outtake timing reset
        if (outtakeShooting && getRuntime() - outtakeStartTime >= outtakeShootTime) {
            outtakeShooting = false;
            OuttakeServo.setPosition(OuttakeServoDefaultAngle);
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        Controller1Init();
        Controller2Init();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Controller1Loop();
            Controller2Loop();

            // Display current draw in milliamps
            telemetry.addData("Left Outtake Motor Current (mA)", LeftOuttakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Right Outtake Motor Current (mA)", RightOuttakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Motor Current (mA)",IntakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Servo Position",IntakeServo.getPosition());
            telemetry.addData("Outtake Servo Position",OuttakeServo.getPosition());
            telemetry.addData("Spindexer Encoder Counts", spinDexerRotation);

            telemetry.update();
        }
    }
}