package org.firstinspires.ftc.teamcode.opMode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.SpeedTunning;


@TeleOp(name = "Shooter Dashboard Test")
public class ShooterDashboardTest extends OpMode {

    SpeedTunning.Shooter shooter;

    @Override
    public void init() {
        SpeedTunning tuning = new SpeedTunning();
        shooter = tuning.new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.update();

        telemetry.addData("Target Velocity", SpeedTunning.Params.Velocity);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
