package org.firstinspires.ftc.teamcode.DECODE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class TunningData extends OpMode {
    private DcMotorEx flyWheel, Intake;
    private Servo hoodAngle;

    public static double velocity = 1650;
    public static double hoodPos = 0.2;

    @Override
    public void init() {
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flywheelPID = new PIDFCoefficients(200, 0, 0, 13.1000);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPID);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
        hoodAngle.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean Y = gamepad1.y;
        boolean A = gamepad1.a;

        if (lt > 0.1) {
            Intake.setPower(1);
        } else if (rt > 0.1) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }

        if (Y) {
            flyWheel.setVelocity(velocity);
        } else {
            flyWheel.setPower(0);
        }
        if (A) {
            hoodAngle.setPosition(hoodPos);
        }
    }
}
