package org.firstinspires.ftc.teamcode.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class RobotConstant {
    public static Pose2d blueAimingTarget = new Pose2d(-57.1, -55.3, 0);
    public static Pose2d redAimingTarget = new Pose2d(-blueAimingTarget.position.x, -blueAimingTarget.position.y, 0);

    public static double[] HoodedAngle = {0.0, 0.9};
    public static double OPEN_GATE = 0.5;
    public static double CLOSE_GATE = 0.9;
    public static double INTAKE_SPEED = 1.0;
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;
    public static double[] TRIANGLE_X = {-55.3, 0, -55.3};
    public static double[] TRIANGLE_Y = {-55.0, 0, 55.0};
    public static double[] TRIANGLE_XS = {};
    public static double[] TRIANGLE_YS = {};
    public static double MIN_ZONE_X = 0;
    public static double MAX_ZONE_X = 144;
    public static double MIN_ZONE_Y = 0;
    public static double MAX_ZONE_Y = 144;

    public double EveryWhereShooInterpolation(double pos) {
        double[][] dataPoints = {
                {0, 0},
                {0, 0}
        };

        if (pos <= dataPoints[0][0]) {
            return dataPoints[0][1];
        }

        if (pos >= dataPoints[dataPoints.length - 1][0]) {
            return dataPoints[dataPoints.length - 1][1];
        }

        for (int i = 0; i < dataPoints.length - 1; i++) {
            double d1 = dataPoints[i][0];
            double p1 = dataPoints[i][1];
            double d2 = dataPoints[i + 1][0];
            double p2 = dataPoints[i + 1][1];

            if (pos >= d1 && pos <= d2) {
                double power = p1 + (p2 - p1) * (pos - d1) / (d2 - d1);

                if (power < 0) power = 0;
                if (power > 3000) power = 3000;

                return power;
            }
        }
        return 1500;
    }

    public double AngleAdjuster(double pos) {
        for (int i = 0; i < HoodedAngle.length; i++) {
            double angle = HoodedAngle[0 + 1];
        }
        return 0.8;
    }

    public double pivotX(double robotX, double robotHeading) {
        return robotX + TURRET_OFFSET_X * Math.cos(robotHeading) - TURRET_OFFSET_Y * Math.sin(robotHeading);
    }

    public double pivotY(double robotY, double robotHeading) {
        return robotY + TURRET_OFFSET_X * Math.sin(robotHeading) + TURRET_OFFSET_Y * Math.cos(robotHeading);
    }
}
