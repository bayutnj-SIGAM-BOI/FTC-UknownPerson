package org.firstinspires.ftc.teamcode.AdvanceCode;

public class HeadingPIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double integralLimit = 30; // cegah integral windup

    public HeadingPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculateInches(double target, double current) {
        double error = target - current;

        if (Math.abs(error) < 3.0) {
            integralSum += error;
        } else {
            integralSum = 0;
        }
        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));

        double derivative = error - lastError;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);
        return output;
    }

    public double calculateRadians(double targetHeading, double currentHeading) {
//        P = Error
        double error = angleWrapRadians(targetHeading - currentHeading);

//        Integral = Disappear Error Kecil
        if (Math.abs(error) < 0.1) {
            integralSum += error;
        } else {
            integralSum = 0;
        }
        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));

//        Derivative = biar gk terlalu kenceng
        double derivative = error - lastError;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);
        return output;
    }

    public double calculateDegree(double targetHeading, double currentHeading) {
        double error = angleWrapDegree(targetHeading - currentHeading);

//        Integral = Disappear Error Kecil
        if (Math.abs(error) < 10.0) {
            integralSum += error;
        } else {
            integralSum = 0;
        }
        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));

//        Derivative = biar gk terlalu kenceng
        double derivative = error - lastError;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);
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
        integralSum = 0;
        lastError = 0;
    }
}
