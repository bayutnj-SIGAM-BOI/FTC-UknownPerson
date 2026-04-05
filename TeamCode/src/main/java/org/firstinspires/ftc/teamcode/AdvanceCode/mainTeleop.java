package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class mainTeleop extends OpMode {
    turret turret = new turret();
    Config r = new Config();

    private boolean lastY = false;
    private boolean lastA = false;
    private boolean stooperOpen = false;
    private boolean ShooterOn = false;

    @Override
    public void init() {
        r.initialize(hardwareMap, true, false, false, true);
        turret.initalize(hardwareMap, telemetry);
        turret.spinTimer.reset();

    }

    @Override
    public void loop() {
        Movement();
        turret.updateWebcam();
        turret.setAngleAdjuster();
        turret.spinningTurret();


        if (gamepad1.y && !lastY) {
            ShooterOn = !ShooterOn;
            if (ShooterOn) turret.setVelocityAuto();
            else turret.turretWheel.setVelocity(0);
        }
        lastY = gamepad1.y;

        if (gamepad1.a && !lastA) {
            stooperOpen = !stooperOpen;
            if (stooperOpen) turret.setStooperOpen();
            else turret.setStooperClose();
        }
        lastA = gamepad1.a;

        if (gamepad1.left_bumper) {
            r.Intake(1.0);
        } else {
            r.Intake(0);
        }

        telemetry.addData("Angle", turret.angle);
        telemetry.addData("Range", turret.range);
        telemetry.addData("Velocity", turret.vel);
        telemetry.addData("Bearing", turret.bearing);
        telemetry.addData("Power Compiled", turret.compiledPower);
        telemetry.update();
    }

    public void Movement() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.right_stick_x;

        double leftPower = Range.clip(y - x, -1.0, 1.0);
        double rightPower = Range.clip(y + x, -1.0, 1.0);

        r.tankMotors(leftPower, rightPower);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
