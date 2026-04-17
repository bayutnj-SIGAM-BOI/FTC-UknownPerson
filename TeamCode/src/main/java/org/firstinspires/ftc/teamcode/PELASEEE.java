package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class PELASEEE extends OpMode {
    DcMotor leftMotor, rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;

        double lp = Range.clip(y - x, -1, 1);
        double rp = Range.clip(y + x, -1, 1);
        leftMotor.setPower(lp);
        rightMotor.setPower(rp);
    }
}
