package org.firstinspires.ftc.teamcode.SeasonTWO;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@com.acmerobotics.dashboard.config.Config
public class Teleop extends OpMode {
    //    Import Components
    Config TelConf;
    HeadingPIDController headingPID;
    HeadingPIDController armPID;
    //    Drive base Config
    DcMotor leftMotor, rightMotor = null;
    IMU imu;
    public static double dKP = 0;
    public static double dKI = 0;
    public static double dKD = 0;

    double targetHeading = 0;
    boolean autoPIDHeading = false;

    //    SubSystem Config
    public static double aKP = 0;
    public static double aKI = 0;
    public static double aKD = 0;
    public static double aKF = 0;

    DcMotorEx armMotor = null;
    public static int target = 0;
    final double ticks = 28 * 40 / 3.0; // Ticks formula (Ticks(28) * Internal ratio * external ratio))

    @Override
    public void init() {
//        Drive base Initialize
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        SubSystem Initialize
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
//        IMU Initialize
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParams);
        imu.resetYaw();
    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            TelConf.resetAndStraightTo(72, 0);
        }

        if (armPID == null) armPID = new HeadingPIDController(aKP, aKI, aKD);
        int currentArmPos = armMotor.getCurrentPosition();
        double currentDegree = (currentArmPos * 360.0) / ticks;

//        buat ngejar target degreenya
        double pid = armPID.calculateDegree(target, currentDegree);
//        Ngasih power extra
        double angleRad = Math.toRadians(target);
        double feedforward = Math.cos(angleRad) * aKF;
        double power = pid + feedforward;
        if (gamepad1.left_bumper) {
            armMotor.setVelocity(power);
        } else {
            armMotor.setPower(0);
        }

        if (headingPID == null) headingPID = new HeadingPIDController(dKP, dKI, dKD);

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.dpadUpWasPressed()) {
            targetHeading = Math.toRadians(0);
            autoPIDHeading = true;
            headingPID.reset();
        } else if (gamepad1.dpadDownWasPressed()) {
            targetHeading = Math.toRadians(120);
            autoPIDHeading = true;
            headingPID.reset();
        } else if (gamepad1.dpadRightWasPressed()) {
            targetHeading = Math.toRadians(-120);
            autoPIDHeading = true;
            headingPID.reset();
        } else if (gamepad1.leftBumperWasPressed()) {
            targetHeading = Math.toRadians(180);
            autoPIDHeading = true;
            headingPID.reset();
        }

        if (Math.abs(gamepad1.left_stick_y) >= 0.3 || Math.abs(gamepad2.left_stick_x) >= 0.1) {
            autoPIDHeading = false;
        }
        double drive = -gamepad1.left_stick_y;
        double rotate;

        if (autoPIDHeading) {
            rotate = headingPID.calculateRadians(targetHeading, currentHeading);
            rotate = Range.clip(rotate, -0.6, 0.6);
        } else {
            rotate = gamepad1.right_stick_x;
        }

        double leftPower = Range.clip(drive + rotate, -1.0, 1.0);
        double rightPower = Range.clip(drive - rotate, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Current Heading", "%.1f°", currentHeading);
        telemetry.addData("Target Heading", "%.1f°", targetHeading);
        telemetry.addData("Auto Heading", autoPIDHeading ? "AUTO" : "MANUALLY");
        telemetry.addData("Left / Right Power", "%.2f / %.2f", leftPower, rightPower);
        telemetry.update();
    }
}
