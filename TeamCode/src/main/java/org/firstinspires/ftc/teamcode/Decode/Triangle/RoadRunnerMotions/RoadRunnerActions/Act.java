package org.firstinspires.ftc.teamcode.Decode.Triangle.RoadRunnerMotions.RoadRunnerActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Decode.Triangle.RobotStatic;

public class Act {
    private final RobotStatic rc;
    private DcMotorEx Intake, Shooter;
    private Servo stooperGate, angleAdjuster;


    public Act(HardwareMap hardwareMap, Telemetry telemetry) {
        rc = new RobotStatic();

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        stooperGate = hardwareMap.get(Servo.class, "Stooper");
        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public Action shooterWheel(double pos) {
        return new shooterWheel(pos);
    }

    public Action Intake() {
        return new IntakeAct();
    }

    public Action StopShooter() {
        return new StopShooter();
    }

    public Action setStooper(double angle) {
        return new setStooperOpen(angle);
    }

    public Action IntakeStop() {
        return new IntakeStop();
    }

    public Action autoAdjust(double d) {
        return new autoAdjust(d);
    }

    public class StopShooter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Shooter.setPower(0);
            return false;
        }
    }

    public class shooterWheel implements Action {
        private boolean initialize = false;
        private double pos;

        public shooterWheel(double pos) {
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialize) {
                Shooter.setVelocity(0);
                initialize = true;
            }
            rc.EveryWhereShooInterpolation(pos);
            return false;
        }
    }

    public class IntakeAct implements Action {
        private boolean initialize = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialize) {
                Intake.setPower(0);
                initialize = true;
            }
            Intake.setPower(1.0);
            return false;
        }
    }

    public class IntakeStop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Intake.setPower(0);
            return false;
        }
    }

    public class setStooperOpen implements Action {
        private boolean initialize = false;
        private final double angle;

        public setStooperOpen(double angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialize) {

                initialize = true;
            }

            stooperGate.setPosition(angle);
            return false;
        }
    }

    public class autoAdjust implements Action {
        private boolean initialize = false;
        private double d;

        public autoAdjust(double d) {
            this.d = d;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialize) {
                initialize = true;
            }

            angleAdjuster.setPosition(rc.AngleAdjuster(d));
            return false;
        }
    }
}
