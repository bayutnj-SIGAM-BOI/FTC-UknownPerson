package org.firstinspires.ftc.teamcode.Decode.Triangle;

public class ableToShootTriangle {
    public boolean InsideField(double robotX, double robotY) {
        return RobotConstant.MIN_ZONE_X <= robotX && RobotConstant.MAX_ZONE_X >= robotX
                && RobotConstant.MIN_ZONE_Y <= robotY && RobotConstant.MAX_ZONE_Y >= robotY;
    }

    public double calculateTrig(double x1, double y1, double x2, double y2, double x3, double y3) {
//        Formula From Google || formula to know triangle point ||
        return (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0;
    }

    public boolean ableToShoot(double RobotX, double RobotY) {
        double lX = RobotConstant.TRIANGLE_X[0];
        double mX = RobotConstant.TRIANGLE_X[1];
        double rX = RobotConstant.TRIANGLE_X[2];

        double lY = RobotConstant.TRIANGLE_Y[0];
        double mY = RobotConstant.TRIANGLE_Y[1];
        double rY = RobotConstant.TRIANGLE_Y[2];

        double p1 = calculateTrig(RobotX, RobotY, lX, lY, mX, mY);
        double p2 = calculateTrig(RobotX, RobotY, mX, mY, rX, rY);
        double p3 = calculateTrig(RobotX, RobotY, rX, rY, lX, lY);

        double lXs = RobotConstant.TRIANGLE_XS[0];
        double mXs = RobotConstant.TRIANGLE_XS[1];
        double rXs = RobotConstant.TRIANGLE_XS[2];

        double lYs = RobotConstant.TRIANGLE_YS[0];
        double mYs = RobotConstant.TRIANGLE_YS[1];
        double rYs = RobotConstant.TRIANGLE_YS[2];

        double ps1 = calculateTrig(RobotX, RobotY, lXs, lYs, mXs, mYs);
        double ps2 = calculateTrig(RobotX, RobotY, mXs, mYs, rXs, rYs);
        double ps3 = calculateTrig(RobotX, RobotY, rXs, rYs, lXs, lYs);

        boolean True = (p1 >= 0 && p2 >= 0 && p3 >= 0) || (ps1 >= 0 && ps2 >= 0 && ps3 >= 0);
        boolean False = (p1 <= 0 && p2 <= 0 && p3 <= 0) || (ps1 <= 0 && ps2 <= 0 && ps3 <= 0);
        boolean insideTriangle = True || False;

        return InsideField(RobotX, RobotY) && insideTriangle;
    }
}
