package org.firstinspires.ftc.teamcode.DECODE;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class HeadingPIDController {
    public double kP, kI, kD;
    private double integralSumInches = 0;
    private double integralSumDegree = 0;
    private double integralSumRadians = 0;

    private double lastErrorDegree = 0;
    private double lastErrorRadians = 0;
    private double lastErrorInch = 0;
    private double integralLimit = 30; // integral windup
    private ElapsedTime RadiansTimer = new ElapsedTime();
    private ElapsedTime degreeTimer = new ElapsedTime();
    private ElapsedTime inchTimer = new ElapsedTime();

    public HeadingPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        RadiansTimer.reset();

        degreeTimer.reset();

        inchTimer.reset();
    }

    public double calculateInches(double target, double current) {
        double error = target - current;

        if (Math.abs(error) > 3.0) {
            integralSumInches += error * inchTimer.seconds();
        } else {
            integralSumInches = 0;
        }
        integralSumInches = Range.clip(integralSumInches, -integralLimit, integralLimit);

        double derivative = (error - lastErrorInch) / inchTimer.seconds();
        lastErrorInch = error;
        inchTimer.reset();

        double output = (error * kP) + (integralSumInches * kI) + (derivative * kD);
        return output;
    }

    public double calculateRadians(double targetHeading, double currentHeading) {
//        P = Error
        double error = angleWrapRadians(targetHeading - currentHeading);

//        Integral = Disappear Error Kecil
        if (Math.abs(error) > 0.1) {
            integralSumRadians += error * RadiansTimer.seconds();
        } else {
            integralSumRadians = 0;
        }
        integralSumRadians = Range.clip(integralSumRadians, -integralLimit, integralLimit);


//        Derivative = biar gk terlalu kenceng
        double derivative = (error - lastErrorRadians) / RadiansTimer.seconds();
        lastErrorRadians = error;
        RadiansTimer.reset();

        double output = (error * kP) + (integralSumRadians * kI) + (derivative * kD);
        return output;
    }

    public double calculateDegree(double targetHeading, double currentHeading) {
        double error = angleWrapDegree(targetHeading - currentHeading);

//        Integral = Disappear Error Kecil
        if (Math.abs(error) > 10.0) {
            integralSumDegree += error * degreeTimer.seconds();
        } else {
            integralSumDegree = 0;
        }
        integralSumDegree = Range.clip(integralSumDegree, -integralLimit, integralLimit);

//        Derivative = biar gk terlalu kenceng
        double derivative = (error - lastErrorDegree) / degreeTimer.seconds();
        lastErrorDegree = error;
        degreeTimer.reset();

        double output = (error * kP) + (integralSumDegree * kI) + (derivative * kD);
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

    public double angleWrapRadians(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public void reset() {
        integralSumInches = 0;
        integralSumDegree = 0;
        integralSumRadians = 0;
        lastErrorDegree = 0;
        lastErrorInch = 0;
        lastErrorRadians = 0;
    }
}
