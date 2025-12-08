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

    final double Spindexder_RPM = 117;
    final double Seconds_Per_Rev = (1/(Spindexder_RPM /60));
    final double Spindexer_move_120 = Seconds_Per_Rev/3;
    final double Servo_Start_Position = 0.05;
    final double IntakeServoEndPosition = 0.2;
    final double OuttakeServoEndPosition = 0.3;

    final double SpindexerEncoderPulsesPerRevolution = (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * 28;


    double spinDexerRotation = 0;



    boolean previousGamepadY = false;
    public void Controller2Init() {
        LeftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "LeftOuttakeMotor");
        RightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "RightOuttakeMotor");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "SpindexerMotor");

        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        OuttakeServo = hardwareMap.get(Servo.class, "OuttakeServo");

        SpindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SpindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeServo.setPosition(Servo_Start_Position);
        OuttakeServo.setPosition(Servo_Start_Position);
    }

    public void Controller2Loop() {
        if (gamepad2.a) {
            LeftOuttakeMotor.setPower(1);
            RightOuttakeMotor.setPower(-1);
            //telemetry.addLine("Power 1");
        } else {
            LeftOuttakeMotor.setPower(0);
            RightOuttakeMotor.setPower(0);
            //telemetry.addLine("Power 0");
        }

        if (gamepad2.x){
            IntakeMotor.setPower(1);
            sleep(1500);
            IntakeMotor.setPower(0);
        }
        if (gamepad2.y && !previousGamepadY){

            // rotates by 1/3 of a total revolution
            spinDexerRotation += SpindexerEncoderPulsesPerRevolution/3;

            SpindexerMotor.setTargetPosition((int) spinDexerRotation);
            SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SpindexerMotor.setPower(0.3);
        }
        if (gamepad2.dpad_up){
            IntakeServo.setPosition(IntakeServoEndPosition);
            sleep(1000);
            IntakeServo.setPosition(Servo_Start_Position);
        }
        if (gamepad2.dpad_down){
            OuttakeServo.setPosition(OuttakeServoEndPosition);
            sleep(1000);
            OuttakeServo.setPosition(Servo_Start_Position);
        }
        previousGamepadY = gamepad1.y;
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