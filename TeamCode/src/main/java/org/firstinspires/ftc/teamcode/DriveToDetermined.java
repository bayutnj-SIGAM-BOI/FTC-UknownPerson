package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DECODE.PIDControl;

public class DriveToDetermined {
    PIDControl pidControl;
    private boolean arrived = false;
    public PoseVelocity2d driveTo(Pose2d target, Pose2d current) {
        double curX = current.position.x;
        double curY = current.position.y;
        double curH = current.heading.toDouble();

        double tX = target.position.x;
        double tY = target.position.y;
        double tH = target.heading.toDouble();

        double eX = tX - curX;
        double eY = tY - curY;
        double dis = Math.hypot(eX, eY);

        double eH = pidControl.angleWrapRadians(tH - curH);
        arrived = dis < 1.5 && Math.abs(eH) < Math.toRadians(2);

        if (arrived) {
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        double xPower = pidControl.calculateInches(tX, curX);
        double yPower = pidControl.calculateInches(tY, curY);
        double hPower = pidControl.calculateRadians(tH, curH);

        double cos = Math.cos(curH);
        double sin = Math.sin(curH);
        double rotX = xPower * cos - yPower * sin;
        double rotY = xPower * sin + yPower * cos;

        rotX = Range.clip(rotX, -1, 1);
        rotY = Range.clip(rotY, -1, 1);
        hPower = Range.clip(hPower, -1,1);
        return  new PoseVelocity2d(new Vector2d(rotX, rotY), hPower);
    }

    public boolean isArrived() {
        return arrived;
    }

}
