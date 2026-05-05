package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "Pinpoint Direction Check")
public class PinpointDirection extends LinearOpMode {
    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-6, 1, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("X", odo.getPosX(DistanceUnit.INCH));
            packet.put("Y", odo.getPosY(DistanceUnit.INCH));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}