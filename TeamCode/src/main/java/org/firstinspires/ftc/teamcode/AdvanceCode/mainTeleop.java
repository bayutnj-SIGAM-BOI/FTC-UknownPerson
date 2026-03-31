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
        r.initialize(hardwareMap, true, false, true, true);
        turret.initalize(hardwareMap);
    }

    @Override
    public void loop() {
        Movement();

        turret.setAngleAdjuster();
        turret.spinningTurret();

        if (gamepad1.right_trigger > 0.5) {
            turret.setVelocityAuto();
        }

        if (gamepad1.a && !lastA) {
            stooperOpen = !stooperOpen;
            turret.setStooperOpen();
        } else if (!gamepad1.a) {
            turret.setStooperClose();
        }
        lastA = gamepad1.a;

        if (gamepad1.y && !lastY) {
            ShooterOn = !ShooterOn;
            r.Shooter(1800);
        } else if (!gamepad1.y) {
            r.Shooter(0);
        }
        lastY = gamepad1.y;
        if (gamepad1.left_trigger > 0.3) {
            r.Intake(1800);
        } else {
            r.Intake(0);
        }

    }

    public void Movement() {
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        double leftPower = Range.clip(forward - rotate, -1.0, 1.0);
        double rightPower = Range.clip(forward + rotate, -1.0, 1.0);

        r.tankMotors(leftPower, rightPower);
    }
}
