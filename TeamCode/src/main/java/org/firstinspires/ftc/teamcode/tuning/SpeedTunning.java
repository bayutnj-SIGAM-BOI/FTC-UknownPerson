package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class SpeedTunning {
    public static class Params {
        public static double Velocity = 1000;
        public static double leftSpeed = 0;
        public static double rightSpeed = 0;

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;

    }

    public static Params PARAMS = new Params();

    public class Shooter {
        DcMotorEx flyWheel;

        public Shooter(HardwareMap hardwareMap) {
            flyWheel = hardwareMap.get(DcMotorEx.class, "flywheel");

            flyWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flyWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Params.kP, Params.kI, Params.kD, Params.kF);
            flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }

        public void update() {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Params.kP, Params.kI, Params.kD, Params.kF);
            flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            flyWheel.setVelocity(Params.Velocity);
        }
        public double getVelocity() {
            return flyWheel.getVelocity();
        }

        public Shooter(DcMotorEx flyWheel) {
            this.flyWheel = flyWheel;
        }

        public void stop() {
            flyWheel.setVelocity(0);
        }
    }

    public final class DriveBase {
        DcMotor LeftMotor, RightMotor;

        public DriveBase(HardwareMap hardwareMap) {
            LeftMotor = hardwareMap.get(DcMotor.class, "left_motor_Drive");
            RightMotor = hardwareMap.get(DcMotor.class, "right_motor_Drive");

            LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void update() {
            LeftMotor.setPower(Params.leftSpeed);
            RightMotor.setPower(Params.rightSpeed);
        }

        public void stop() {
            LeftMotor.setPower(0);
            RightMotor.setPower(0);
        }
    }
}
