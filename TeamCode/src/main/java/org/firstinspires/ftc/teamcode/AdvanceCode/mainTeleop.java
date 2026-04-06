package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class mainTeleop extends OpMode {

    turret turret = new turret();
    Config r = new Config();
    HeadingPIDController angleCorrection;
    IMU imu;

    private boolean lastY = false;
    private boolean lastA = false;
    private boolean stooperOpen = false;
    private boolean ShooterOn = false;

    double leftPower = 0.0;
    double rightPower = 0.0;

    @Override
    public void init() {
        r.initialize(hardwareMap, true, true, false, false, true);
        turret.initalize(hardwareMap, telemetry);
        turret.spinTimer.reset();
        angleCorrection = new HeadingPIDController(2, 0, 0.005);
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
        telemetry.addData("Range", turret.range != null ? turret.range.ftcPose.range : "null");
        telemetry.addData("Angle", turret.angle);
        telemetry.addData("Shooter Status", ShooterOn ? "Shooter ON" : "Shooter OFF");
        telemetry.addData("Shooter Velocity", turret.vel);
        telemetry.addData("Power Compiled", turret.compiledPower);

        telemetry.addLine("-------- Intake --------");
        telemetry.addData("Intake Velocity", r.Intake.getVelocity());

        telemetry.addLine("-------- Drive base --------");
        telemetry.addData("Robot Orientation", turret.currentRobotYaw);
        telemetry.addData("Left Side", leftPower);
        telemetry.addData("Right Side", rightPower);

        telemetry.update();
    }

    public void Movement() {
        double current = r.getHeading(); // we track our current degree
        double TargetHeading = Math.toRadians(90); // In degree

        double y = -gamepad1.left_stick_y;
        double x;

        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            x = -gamepad1.right_stick_x;
        } else {
            x = angleCorrection.calculateRadians(TargetHeading, current);
        }

        this.leftPower = Range.clip(y - x, -1.0, 1.0);
        this.rightPower = Range.clip(y + x, -1.0, 1.0);

        r.tankMotors(leftPower, rightPower);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
