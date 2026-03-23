package org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner.Component;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ShooterAction {
    public DcMotorEx flyWheel = null;
    public Servo Launcher;

    public ShooterAction(HardwareMap hardwareMap) {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        Launcher = hardwareMap.get(Servo.class, "Launcher");

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Launcher.setPosition(0.875);
    }

    public class SpinUp implements Action {
        private boolean initialize = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialize) {
                flyWheel.setVelocity(1650);
                initialize = true;
            }
            double vel = flyWheel.getVelocity();
            packet.put("Shoot Vel", vel);
            return Math.abs(vel - 1650) > 50;
        }
    }

    public Action SpinUp() {
        return new SpinUp();
    }

    public class Launch implements Action {
        private boolean initialize = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialize) {
                Launcher.setPosition(0.5);
                initialize = true;
            }

            packet.put("Launcher", "ON");
            return false;
        }
    }

    public Action Launch() {
        return new Launch();
    }

    public class stopLaunch implements Action {

        private boolean initialize = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialize) {
                Launcher.setPosition(0.875);
                initialize = true;
            }

            packet.put("Launcher", "OFF");
            return false;
        }
    }

    public Action stopLaunch() {
        return new stopLaunch();
    }

    public Action multiShoot(int count, double delay) {
        Action[] shots = new Action[count * 4];
        int i = 0;

        for (int n = 0; n < count; n++) {
            shots[i++] = Launch();
            shots[i++] = new SleepAction(delay);
            shots[i++] = stopLaunch();
            shots[i++] = new SleepAction(1.000);

        }

        return new SequentialAction(shots);
    }

    public Action stopAction() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    flyWheel.setVelocity(0);
                    Launcher.setPosition(0.875);
                    initialized = true;
                }

                packet.put("Shooter", "STOPPED");
                return false;
            }
        };
    }
}
