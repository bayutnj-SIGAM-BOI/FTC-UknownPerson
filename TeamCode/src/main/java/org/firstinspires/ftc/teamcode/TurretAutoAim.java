package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
import org.firstinspires.ftc.teamcode.DECODE.Turret.TurretWithPoseEstimate;

@Config
@TeleOp
public class TurretAutoAim extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private Servo angleAdjuster, stooperGate;
    private DcMotorEx Shooter;
    private TankDrive drive;
    private TurretWithPoseEstimate turret;
    private final RobotStatic rC = new RobotStatic();

    // ── Targets ───────────────────────────────────────────────────────────────
    private final Pose2d blueAimingTarget = new Pose2d(new Vector2d(-66.4, -59.5), 0);
    private final Pose2d redAimingTarget = new Pose2d(new Vector2d(57.1, 55.3), 0);
    private Pose2d target = blueAimingTarget;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFound = false;
    private boolean lastY = false;
    private boolean shoot = false;

    // ── Auto-aim robot heading PID ────────────────────────────────────────────
    // Tune these from FTC Dashboard
    public static double kP = 0.8;
    public static double kI = 0.001;
    public static double kD = 0.06;

    private static final double HEADING_TOLERANCE = 0.03;  // ~1.7°
    private static final double INTEGRAL_LIMIT = 1.0;
    private static final double MIN_POWER = 0.05;

    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Auto-aim toggle: gamepad1 X
    private boolean autoAimEnabled = false;
    private boolean lastXButton = false;

    // ── Init ──────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        drive = new TankDrive(hardwareMap, new Pose2d(new Vector2d(55.1, -9.1), 0));
        turret = new TurretWithPoseEstimate(hardwareMap);

        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");
        stooperGate = hardwareMap.get(Servo.class, "Stooper");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        pidTimer.reset();
    }

    // ── Loop ──────────────────────────────────────────────────────────────────
    @Override
    public void loop() {

        // ── Localization ─────────────────────────────────────────────────────
        drive.localizer.update();
        Pose2d pose = drive.localizer.getPose();

        double robotX = pose.position.x;
        double robotY = pose.position.y;
        double heading = pose.heading.toDouble();

        // ── Auto-aim toggle (gamepad1 X, edge-detected) ──────────────────────
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            autoAimEnabled = !autoAimEnabled;
            if (autoAimEnabled) {
                integralSum = 0;
                lastError = 0;
                pidTimer.reset();
            }
        }
        lastXButton = xPressed;

        // ── Target selection (bumpers) ────────────────────────────────────────
        isFound = false;
        if (gamepad1.left_bumper) {
            target = blueAimingTarget;
            isFound = true;
        } else if (gamepad1.right_bumper) {
            target = redAimingTarget;
            isFound = true;
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        if (autoAimEnabled && isFound) {
            // Robot rotates to face target — driver still controls forward/back
            double forward = gamepad1.left_stick_y;

            double dx = target.position.x - robotX;
            double dy = target.position.y - robotY;
            double angleToTarget = Math.atan2(dy, dx);
            double error = normalizeAngle(angleToTarget - heading);
            double turnPower = calculatePID(error);

            if (Math.abs(error) < HEADING_TOLERANCE) {
                // Facing target — only allow forward/back
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), 0));
            } else {
                // Spin to face target, driver can still drive forward
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), turnPower));
            }

        } else {
            // Full manual control
            integralSum = 0;
            lastError = 0;

            double x = gamepad1.right_stick_x;
            double y = -gamepad1.left_stick_y;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, 0), x));
        }

        // ── Turret aiming ─────────────────────────────────────────────────────
        if (isFound) {
            turret.aimingTurret(target, robotX, robotY, heading);
        }

        // ── Hood angle ────────────────────────────────────────────────────────
        double distanceToTarget = Math.hypot(
                robotX - target.position.x,
                robotY - target.position.y
        );
        angleAdjuster.setPosition(rC.AngleAdjuster(distanceToTarget));

        // ── Gate ─────────────────────────────────────────────────────────────
        stooperGate.setPosition(gamepad1.a ? RobotStatic.OPEN_GATE : RobotStatic.CLOSE_GATE);

        // ── Shooter (gamepad1 Y toggle, edge-detected) ────────────────────────
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shoot = !shoot;
            Shooter.setVelocity(shoot ? rC.EveryWhereShooInterpolation(distanceToTarget) : 0);
        }
        lastY = yPressed;

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Mode", autoAimEnabled ? "AUTO-AIM (X to toggle)" : "MANUAL (X to toggle)");
        telemetry.addData("Target", isFound ? (gamepad1.left_bumper ? "BLUE" : "RED") : "none");
        telemetry.addLine("---");
        telemetry.addData("X", "%.1f", robotX);
        telemetry.addData("Y", "%.1f", robotY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(heading));
        telemetry.addData("Dist target", "%.1f in", distanceToTarget);
        telemetry.addData("Shoot", shoot ? "ON" : "off");
        telemetry.update();
    }

    // ── PID calculator ────────────────────────────────────────────────────────
    private double calculatePID(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) < HEADING_TOLERANCE) {
            integralSum = 0;
            lastError = 0;
            return 0;
        }

        // Only integrate when close to avoid windup
        if (Math.abs(error) < Math.toRadians(15)) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Range.clip(output, -1.0, 1.0);

        // Minimum power to overcome static friction
        if (Math.abs(output) < MIN_POWER && Math.abs(error) > HEADING_TOLERANCE) {
            output = MIN_POWER * Math.signum(output);
        }

        return output;
    }

    // ── Normalize angle to ±π ─────────────────────────────────────────────────
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}