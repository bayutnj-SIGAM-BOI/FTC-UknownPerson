package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Disabled
public class mainTeleop extends OpMode {

    turret turret = new turret();
    Config r = new Config();
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean stooperOpen = false;
    private boolean ShooterOn = false;

    double leftPower = 0.0;
    double rightPower = 0.0;

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

        telemetry.addLine("-------- Turret Orientation --------");
        telemetry.addData("Bearing", turret.bearing != null ? turret.bearing.ftcPose.bearing : "null");
        telemetry.addData("CurTicks", turret.spinTurret.getCurrentPosition());
        telemetry.addData("Current Target Angle (Radians)", turret.currentTurretAngle);
        telemetry.addData("Error", turret.error);

        telemetry.addLine("-------- Shooter & AdjustAngle --------");
        telemetry.addLine("Press Y Button");
        telemetry.addData("Range", turret.range != null ? turret.range.ftcPose.range : "null");
        telemetry.addData("Angle", turret.angle);
        telemetry.addData("Shooter Status", ShooterOn ? "Shooter ON" : "Shooter OFF");
        telemetry.addData("Shooter Velocity", turret.vel);
        telemetry.addData("Power Compiled", turret.compiledPower);

        telemetry.addLine("-------- Intake --------");
        telemetry.addLine("Press LB ( Left Bumper )");
        telemetry.addData("Intake Velocity", r.Intake.getVelocity());

        telemetry.addLine("-------- Drive base --------");
        telemetry.addLine("Forward ( Left Stick Y-Axis) | Rotate ( Right Stick X-Axis ) \n Heading correction work if Stick X >! 0.1");
        telemetry.addData("Robot Orientation", turret.currentRobotYaw);
        telemetry.addData("Left Side", leftPower);
        telemetry.addData("Right Side", rightPower);

        telemetry.update();
    }

    public void Movement() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.right_stick_x;

        this.leftPower = Range.clip(y - x, -1.0, 1.0);
        this.rightPower = Range.clip(y + x, -1.0, 1.0);

        r.tankMotors(leftPower, rightPower);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
