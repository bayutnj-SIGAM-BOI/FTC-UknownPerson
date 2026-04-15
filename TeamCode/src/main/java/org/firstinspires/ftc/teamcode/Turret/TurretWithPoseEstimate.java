package org.firstinspires.ftc.teamcode.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class TurretWithPoseEstimate {
    final RobotConstant rC = new RobotConstant();
    DcMotorEx spinTurret;
    ElapsedTime spinTimer = new ElapsedTime();
    private final double TICKS_PER_REV = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    private final double GearRatio = 5.0;
    private final double OutputSpeed = TICKS_PER_REV * GearRatio;
    final double TicksPerDegree = OutputSpeed / 360.0;
    private final double turretLimitDeg = 135.0;
    private double integralSum = 0;
    private double lastError = 0;
    private final double integralLimit = 15.0;
    public static double kP = 0.05;
    public static double kI = 0.001;

    public static double kD = 0.003;

    private final double maxLimit = 90;
    private final double MinLimit = -120;

    public TurretWithPoseEstimate(HardwareMap hardwareMap) {
        spinTurret = hardwareMap.get(DcMotorEx.class, "spinTurret");
        spinTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinTimer.reset();
    }

    public void aimingTurret(Pose2d Target, double RobotX, double RobotY, double robotHeading) {
        double pivotX = rC.pivotX(RobotX, robotHeading);
        double pivotY = rC.pivotY(RobotY, robotHeading);

        double x = Target.position.x - pivotX;
        double y = Target.position.y - pivotY;


        double angle = Math.atan2(y, x); // Vector(Pose) -> Radians
        double turret = Math.toDegrees(angle) - robotHeading;

//        Limit rotasinya
        turret = Range.clip(turret, MinLimit, maxLimit);

        double currentDeg = spinTurret.getCurrentPosition() / TicksPerDegree;
        double error = angleWrapDegree(turret - currentDeg);
        error = Range.clip(error, -turretLimitDeg, turretLimitDeg);

        if (currentDeg > maxLimit + 5 || currentDeg < MinLimit - 5) {
            spinTurret.setPower(0);
            return;
        }

        spinTurret.setPower(Range.clip(calculatePID(error), -1, 1));
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) < 3.0) {
            integralSum = 0.0;
            lastError = 0.0;
            return 0;
        }

        if (Math.abs(error) < 10.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);
//        klo ada target miss kecil bisa dibenerin klo si motor gk kuat
        if (Math.abs(output) < 0.5 && Math.abs(error) > 0.3) {
            output = 0.05 * Math.signum(output);
        }

        return output;
    }

    //    Fungsi yang membantu agar tidak kebanyakan muter.
    private double angleWrapDegree(double degree) {
        while (degree > 180) {
            degree -= 360;
        }
        while (degree < -180) {
            degree += 360;
        }
        return degree;
    }


}
