package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class turret {
    org.firstinspires.ftc.teamcode.AdvanceCode.Config r = new org.firstinspires.ftc.teamcode.AdvanceCode.Config();
    AprilTagWebcam webcam = new AprilTagWebcam();

    DcMotorEx spinTurret, turretWheel;
    Servo angleAdjuster, Stooper;
    VoltageSensor myVoltageSensor;
    AprilTagDetection aprilTagDetection;
    private Telemetry telemetry;

    //    Telemetry
    public double angle = 0.0;
    public double compiledPower = 0.0;
    public AprilTagDetection range;
    public AprilTagDetection bearing;
    public double vel = 0.0;
    public double error = 0.0;
    public double currentRobotYaw = 0.0;
    public double currentTurretAngle = 0.0;

//    Params Dashboard

    public static double turretWheelP = 8.0;
    public static double turretWheelI = 0;
    public static double turretWheelD = 0.5;
    public static double turretWheelF = 12.5;

    public static double spinTurretP = 0.15;
    public static double spinTurretI = 0;
    public static double spinTurretD = 0.2;

    //    turret & Shooter variables
    public static double nearAngle = 0.5;
    public static double farAngle = 0.0;
    private double integralSum = 0;
    private double lastError = 0;
    private double integralLimit = 15.0;
    private double goalX = 0.0;
    private double angleTolerance = Math.toRadians(2);
    public ElapsedTime spinTimer = new ElapsedTime();

    IMU imu;
    private double ticks_per_rev = 537.6898395722 * (100.0 / 20.0);
    private double ticksToRad = (2 * Math.PI) / ticks_per_rev;

    public void initalize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        webcam.init(hardwareMap, telemetry);

        turretWheel = hardwareMap.get(DcMotorEx.class, "turretWheel");
        turretWheel.setDirection(DcMotorSimple.Direction.FORWARD); // FORWARD
        turretWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(turretWheelP, turretWheelI, turretWheelD, turretWheelF);
        turretWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        spinTurret = hardwareMap.get(DcMotorEx.class, "spinTurret");
        spinTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");
        angleAdjuster.setPosition(0.8);
        Stooper = hardwareMap.get(Servo.class, "Stooper");

//        myVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        myVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void manualTurret(double gpad) {
        double power = Range.clip(gpad, -1, 1);
        spinTurret.setPower(power);
    }

    public void updateWebcam() {
        webcam.update();
    }

    public AprilTagDetection setWebcam() {
        AprilTagDetection id20 = webcam.getTagBySpesificId(20);
        AprilTagDetection id24 = webcam.getTagBySpesificId(24);

        AprilTagDetection isTargetFound = null;

        if (id20 != null) {
            isTargetFound = id20;
        } else if (id24 != null) {
            isTargetFound = id24;
        }
        return isTargetFound;
    }

    public void setVelocityAuto() {
        AprilTagDetection target = setWebcam();

        if (target == null) {
            r.Shooter(1500);
            return;
        }

        double batteryVoltage = myVoltageSensor.getVoltage();
        vel = calculateLauncherPower(target.ftcPose.range);
        compiledPower = batteryCompiled(this.vel, batteryVoltage);
        turretWheel.setVelocity(this.compiledPower);
    }

    public double batteryCompiled(double basePower, double volt) {
        double compensatedPower = basePower;
//        battery threshold
//        semakin rendah voltage kompensasinya lebih gede
        if (volt < 12.3) {
            double voltageDrop = 13.0 - volt;
            double compensation = 1.0 + (voltageDrop * 0.05);

            compensatedPower = basePower * compensation;
        }

        if (compensatedPower < 0) compensatedPower = 0;
        if (compensatedPower > 3000) compensatedPower = 3000;

        return compensatedPower;

    }

    private double calculateLauncherPower(double distance) {
//        Array pertama jarak dalam bentuk cm.
        double[][] dataPoints = {
                {130, 1450},
                {140, 1550},
                {150, 1800},
                {160, 1900}
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
        Stooper.setPosition(0.87);
    }

    public void setStooperClose() {
        Stooper.setPosition(0.6);
    }

    public void setAngleAdjuster() {
        AprilTagDetection isTargetFound = setWebcam();

        int nearSide = 130;
        int farSide = 170;

        if (isTargetFound == null) {
            angleAdjuster.setPosition(0.8);
            return;
        }

        if (isTargetFound.ftcPose.range < nearSide) {
            this.angle = nearAngle;
        } else if (isTargetFound.ftcPose.range > farSide) {
            this.angle = farAngle;
        } else {
//          biar jika ditengah jarak kisaran 72-144
            double t = (isTargetFound.ftcPose.range - nearSide) / (farSide - nearSide);
            this.angle = nearAngle + (farAngle - nearAngle) * t;
        }

        angleAdjuster.setPosition(this.angle);

    }

    public void spinningTurret() {
        bearing = setWebcam();
        double power;
        double error;

        if (bearing == null) {
            spinTurret.setPower(0);
            integralSum = 0.0;
            return;
        }

        this.currentTurretAngle = spinTurret.getCurrentPosition() * ticksToRad;
        double targetAngle;
        if (bearing != null) {
            targetAngle = angleRadians(Math.toRadians(goalX) - Math.toRadians(bearing.ftcPose.bearing));
        } else {
            this.currentRobotYaw = r.getHeading();
            targetAngle = angleRadians(-currentRobotYaw);

        }
        error = angleRadians(targetAngle - currentTurretAngle);
        this.error = error;

        if (Math.abs(error) < angleTolerance) {
            integralSum = 0;
            lastError = 0;
            spinTurret.setPower(0);
            return;
        }

        power = Range.clip(calculatePID(error), -0.6, 0.6);

        spinTurret.setPower(power);
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        error = angleRadians(error);
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) > angleTolerance) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * spinTurretP) + (integralSum * spinTurretI) + (derivative * spinTurretD);
        return output;
    }

    public double angleRadians(double turretAngle) {
        while (turretAngle > Math.PI) {
            turretAngle -= 2 * Math.PI;
        }
        while (turretAngle < -Math.PI) {
            turretAngle += 2 * Math.PI;
        }
        return turretAngle;
    }

    public void stop() {
        webcam.stop();
    }
}
