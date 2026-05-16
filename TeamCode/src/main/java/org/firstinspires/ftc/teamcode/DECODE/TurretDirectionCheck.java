package org.firstinspires.ftc.teamcode.DECODE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TurretDirectionCheck extends OpMode {
    private DcMotor turret;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x * 0.3;
        turret.setPower(lx);

        telemetry.addData("ENC", turret.getCurrentPosition());
    }
}
