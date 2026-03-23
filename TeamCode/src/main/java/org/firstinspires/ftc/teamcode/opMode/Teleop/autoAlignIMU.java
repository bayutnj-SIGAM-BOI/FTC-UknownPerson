package org.firstinspires.ftc.teamcode.opMode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class autoAlignIMU extends OpMode {

    DcMotor leftDrive, rightDrive = null;
    IMU imu;

    public static double kP = 0;
    public static double targetHeading = 0;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    private double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    private double getRotationPower () {
        double currentHeading = getHeading(AngleUnit.DEGREES);

        double error = targetHeading - currentHeading;

        while (error > 180) { error -= 360; }
        while (error < 180) { error += 360; }

        double power = (kP * error);

        power = Range.clip(power, -7, 7);

        if (Math.abs(error) < 4) {
            power = 0;
        }
        return power;
    }


    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0.1 && gamepad1.right_trigger < 0.5) {
            if (gamepad1.right_trigger > 0.1) {
                targetHeading = 0;
            }
        }
        if (gamepad1.left_trigger > 0.1 && gamepad1.left_trigger < 0.5) {
            if (gamepad1.left_trigger > 0.1) {
                targetHeading = 90;
            }
        }

        telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
        packet.put("Heading", getHeading(AngleUnit.DEGREES));
        dashboard.sendTelemetryPacket(packet);


    }
}
