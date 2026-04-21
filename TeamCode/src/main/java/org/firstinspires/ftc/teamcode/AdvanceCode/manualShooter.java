package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
@Disabled
public class manualShooter extends OpMode {
    DcMotorEx turretWheel;
    public static double shootervel = 1800;

    @Override
    public void init() {
        turretWheel = hardwareMap.get(DcMotorEx.class, "turretWheel");
        turretWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        turretWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            turretWheel.setVelocity(shootervel);
        } else {
            turretWheel.setVelocity(0);
        }
    }
}
