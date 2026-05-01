package org.firstinspires.ftc.teamcode.DECODE.Basic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class MecanumTeleop extends OpMode {
    DcMotor leftFront, rearLeft, rightFront, rearRight;
    DcMotor Intake;
    DcMotorEx flyWheel;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

        rightFront =  hardwareMap.get(DcMotor.class, "rightFront");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flywheelPID =  new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPID);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 1.1;

        double clamp = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower((y + x + rx) / clamp);
        rearLeft.setPower((y - x + rx) / clamp);
        rightFront.setPower((y - x - rx) / clamp);
        rearRight.setPower((y + x - rx) / clamp);

        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean Y = gamepad1.y;
        if (lt > 0.1) {Intake.setPower(1);} else if(rt > 0.1) {Intake.setPower(-1);} else {Intake.setPower(0);}
        if (Y) {flyWheel.setVelocity(2050);} else {flyWheel.setPower(0);}
    }
}
