package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TankDrive;


@TeleOp
@com.acmerobotics.dashboard.config.Config
public class turretPose extends OpMode {
    TankDrive Drive;

    //    Ini bisa diambil dari MeepMeep
    private final Pose2d blueAimingTarget = new Pose2d(-57.1, 55.3, 0);
    //    private final double TargetX = -57.1;
//    private final double TargetY = 55.3;
    ElapsedTime spinTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double integralLimit = 15.0;
    Config r = new Config();
    double leftPower = 0.0;
    double rightPower = 0.0;

    DcMotorEx spinTurret;
    private final double TICKS_PER_REV = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    private final double GearRatio = 5.0;
    private final double OutputSpeed = TICKS_PER_REV * GearRatio;
    private final double TicksPerDegree = OutputSpeed / 360.0;
    private final double turretLimitDeg = 135.0;
    private final int HomePos = 0;
    public static double kP = 0.0;
    public static double kI = 0.0;

    public static double kD = 0.0;

    public static double kF = 0.0;


    @Override
    public void init() {
        Pose2d PoseBegin = new Pose2d(0, 0, 0);
        Drive = new TankDrive(hardwareMap, PoseBegin);

        r.initialize(hardwareMap, true, false, false, true);

        spinTurret = hardwareMap.get(DcMotorEx.class, "spinTurret");
        spinTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        Drive.updatePoseEstimate();

        Pose2d pose = Drive.localizer.getPose();
        double RobotX = pose.position.x;
        double RobotY = pose.position.y;
        double Heading = Math.toDegrees(pose.heading.toDouble());

        aimingTurret(RobotX, RobotY, Heading);

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.right_stick_x;

        this.leftPower = Range.clip(y - x, -1.0, 1.0);
        this.rightPower = Range.clip(y + x, -1.0, 1.0);

        r.tankMotors(leftPower, rightPower);


    }

    private void aimingTurret(double RobotX, double RobotY, double robotHeading) {
        double x = blueAimingTarget.position.x - RobotX;
        double y = blueAimingTarget.position.y - RobotY;

        double angle = Math.atan2(x, y);
        double targetAngle = angleWrapDegree(Math.toDegrees(angle) - robotHeading);
        double currentDeg = spinTurret.getCurrentPosition() / TicksPerDegree;
        double error = targetAngle - currentDeg;

        spinTurret.setPower(Range.clip(calculatePID(error), -1, 1));
    }

    private int ticksToDeg(double deg) {
        double clamp = Range.clip(deg, -turretLimitDeg, turretLimitDeg);
        return (int) Math.round(clamp * TicksPerDegree) + HomePos;
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) > 3) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD) + kF;
        return output;
    }

    public double angleWrapDegree(double degree) {
        while (degree > 180) {
            degree -= 360;
        }
        while (degree <= -180) {
            degree += 360;
        }
        return degree;
    }
}
