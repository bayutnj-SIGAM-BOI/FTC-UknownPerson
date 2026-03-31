package org.firstinspires.ftc.teamcode.AdvanceCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class turret {
    AprilTagWebcam webcam = new AprilTagWebcam();

    DcMotorEx spinTurret, turretWheel;
    Servo angleAdjuster, Stooper;

    public static double turretWheelP = 0;
    public static double turretWheelI = 0;
    public static double turretWheelD = 0;
    public static double turretWheelF = 0;

    public static double spinnTurretP = 0;
    public static double spinnTurretI = 0;
    public static double spinnTurretD = 0;

    private double integralSum = 0;
    private double lastError = 0;
    private double integralLimit = 30.0;
    private double goalX = 0;
    private double angleTolerance = 0.4;
    private ElapsedTime spinTimer = new ElapsedTime();

    public void initalize(HardwareMap hardwareMap) {
        webcam.init(hardwareMap, telemetry);

        turretWheel = hardwareMap.get(DcMotorEx.class, "turretWheel");
        turretWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(turretWheelP, turretWheelI, turretWheelD, turretWheelF);
        turretWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        spinTurret = hardwareMap.get(DcMotorEx.class, "spinTurret");
        spinTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");
        Stooper = hardwareMap.get(Servo.class, "Stooper");
    }

    public AprilTagDetection setWebcam() {
        AprilTagDetection id20 = webcam.getTagBySpesificId(20);
        AprilTagDetection id21 = webcam.getTagBySpesificId(21);

        AprilTagDetection isTargetFound = null;

        if (id20 != null) {
            isTargetFound = id20;
        } else if (id21 != null) {
            isTargetFound = id21;
        }
        return isTargetFound;
    }

    public void setVelocityAuto() {
        webcam.update();
        AprilTagDetection target = setWebcam();

        if (target == null) return;

        double vel = calculateLauncherPower(target.ftcPose.range);
        turretWheel.setVelocity(vel);
    }

    private double calculateLauncherPower(double distance) {
        double[][] dataPoints = {
                {72, 1450},
                {108, 1550},
                {144, 1800},
        };

        if (distance <= dataPoints[0][0]) {
            return dataPoints[0][1];
        }

        if (distance >= dataPoints[dataPoints.length - 1][0]) {
            return dataPoints[dataPoints.length - 1][1];
        }

        for (int i = 0; i < dataPoints.length - 1; i++) {
            double distance1 = dataPoints[i][0];
            double power1 = dataPoints[i][1];

            double distance2 = dataPoints[i + 1][0];
            double power2 = dataPoints[i + 1][1];

            if (distance >= distance1 && distance <= distance2) {
                double vel = power1 + (power2 - power1) * (distance - distance1) / (distance2 - distance1);

                if (vel < 0) vel = 0;
                if (vel > 3000) vel = 3000;
                return vel;
            }
        }
        return 1500;
    }

    public void setStooperOpen() {
        Stooper.setPosition(0.0);
    }

    public void setStooperClose() {
        Stooper.setPosition(1.0);
    }

    public void setAngleAdjuster() {
        webcam.update();

        AprilTagDetection isTargetFound = setWebcam();

        int nearSide = 72;
        int farSide = 144;
        double angle;

        if (isTargetFound != null) {
            if (isTargetFound.ftcPose.range < nearSide) {
                angle = 0.5;
            } else if (isTargetFound.ftcPose.range > farSide) {
                angle = 0.1;
            } else {
//                biar jika ditengah jarak kisaran 72-144
                double t = (isTargetFound.ftcPose.range - nearSide) / (farSide - nearSide);
                angle = 0.5 - (0.4 * t);
            }
            angleAdjuster.setPosition(angle);
        }
    }

    public void spinningTurret() {
        webcam.update();

        AprilTagDetection target = setWebcam();

        double power = 0.0;
        if (target != null) {
            double error = angleRadians(goalX - target.ftcPose.bearing);

            if (Math.abs(error) < angleTolerance) {
                integralSum = 0;
                lastError = 0;
                spinTurret.setPower(0);
                return;
            }

            double pidOutput = calculatePID(goalX, target.ftcPose.bearing);

            power = Range.clip(pidOutput, -0.2, 0.2);
        }
        spinTurret.setPower(power);
    }

    private double calculatePID(double target, double currTarget) {
        double error = angleRadians(target - currTarget);

        if (Math.abs(error) > angleTolerance) {
            integralSum += error * spinTimer.seconds();
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / spinTimer.seconds();
        lastError = error;

        spinTimer.reset();

        double output = (error * spinnTurretP) + (integralSum * spinnTurretI) + (derivative * spinnTurretD);
        return output;
    }

    public double angleRadians(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
