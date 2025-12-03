package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class MotorTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int Spindexer_Position = 475;
        int Spindexer_Start = 0;
        // Use DcMotorEx for advanced features like current sensing
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "BackRightMotor");
        DcMotorEx LeftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "LeftOuttakeMotor");
        DcMotorEx RightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "RightOuttakeMotor");
        DcMotorEx IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        DcMotorEx SpindexerMotor = hardwareMap.get(DcMotorEx.class, "SpindexerMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SpindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SpindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SpindexerMotor.setTargetPosition(Spindexer_Start);
        SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            SpindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.a) {
                LeftOuttakeMotor.setPower(1);
                RightOuttakeMotor.setPower(-1);
                //telemetry.addLine("Power 1");
            } else {
                LeftOuttakeMotor.setPower(0);
                RightOuttakeMotor.setPower(0);
                //telemetry.addLine("Power 0");
            }

            if (gamepad1.x){
                IntakeMotor.setPower(1);
                sleep(1500);
                IntakeMotor.setPower(0);
            }
            if (gamepad1.y){

                SpindexerMotor.setTargetPosition(475);
                SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SpindexerMotor.setPower(1);

            }



            // Display current draw in milliamps
            telemetry.addData("Left Outtake Motor Current (mA)", LeftOuttakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Right Outtake Motor Current (mA)", RightOuttakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Motor Current (mA)",IntakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Spindexer desired position", SpindexerMotor.getTargetPosition());
            telemetry.addData("Spindexer current position", SpindexerMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
