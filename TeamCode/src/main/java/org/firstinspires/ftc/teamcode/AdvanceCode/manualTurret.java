package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class manualTurret extends OpMode {
    DcMotorEx spinTurret;

    @Override
    public void init() {
        spinTurret = hardwareMap.get(DcMotorEx.class, "spinTurret");
        spinTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        spinTurret.setPower(gamepad1.left_stick_y);

        double currentPosition = spinTurret.getCurrentPosition();
        telemetry.addData("curpos", currentPosition);
        telemetry.update();

    }
}
