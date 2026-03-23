package org.firstinspires.ftc.teamcode.SeasonTWO;

import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.acmerobotics.dashboard.config.Config
public class Config {
    //    Import Components
    HeadingPIDController armPID;
    HeadingPIDController straightPID;
    HeadingPIDController turnPID;
    //    Drive base Config
    DcMotor leftMotor, rightMotor = null;
    public static double dKP = 0;
    public static double dKI = 0;
    public static double dKD = 0;
    //    Turn PID
    public static double tKP = 0;
    public static double tKI = 0;
    public static double tKD = 0;
    //    Drive Constants
    final double WHEEL_DIAMETER_INCHES = 3.54331;
    final double TICKS_PER_REV = 28 * (40 / 3.0 / 4.0);
    final double INCHES_PER_TICKS = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    final double STRAIGHT_TOLERANCE_INCHES = 0.5;
    final double TURN_TOLERANCE_DEGREE = 1.5;
    final double MAX_DRIVE_POWER = 1.0;
    final double MIN_DRIVE_POWER = 0.6;
    final double MAX_TURN_POWER = 1.0;
    final double MIN_TURN_POWER = 0.6;

    // SubSystem Config
    DcMotorEx armMotor = null;
    public static double aKP = 0;
    public static double aKI = 0;
    public static double aKD = 0;
    public static double aKF = 0;
    final double ticks = 28 * 40 / 3.0; // Ticks formula (Ticks(28) * Internal ratio * external ratio))

    //    IMU
    IMU imu;
    boolean IMUAvaiable = false;

    public void initialize(HardwareMap hardwareMap) {
//        Drive base initialize
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SubSystem initialize
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

//        IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParams);
        imu.resetYaw();

    }

    public void armTo(int targetDegree) {
        if (armPID == null) armPID = new HeadingPIDController(aKP, aKI, aKD);
        int currentArmPos = armMotor.getCurrentPosition();
        double currentDegree = (currentArmPos * 360.0) / ticks;

//        buat ngejar target degreenya
        double pid = armPID.calculateDegree(targetDegree, currentDegree);
//        Ngasih power extra
        double angleRad = Math.toRadians(targetDegree);
        double feedforward = Math.cos(angleRad) * aKF;
        double power = pid + feedforward;
        armMotor.setVelocity(power * 60);
    }

    public boolean straightTo(double inches, double heading) {
        if (straightPID == null) straightPID = new HeadingPIDController(dKP, dKI, dKD);
        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);

        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double currentInches = (leftInches + rightInches) / 2.0;
        double error = inches - currentInches;

//        ini kalau error udah kurang dari 0.5inches di stop tuh motor nya
        if (Math.abs(error) < STRAIGHT_TOLERANCE_INCHES) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            straightPID = null;
            turnPID = null;
            return true;
        }
        double drivePower = straightPID.calculateDegree(inches, currentInches);
        drivePower = Range.clip(drivePower, -MIN_DRIVE_POWER, MAX_DRIVE_POWER);

//        Ini tuh biar kecepatanya jika kurang dari 0.6 langsung dibulatkan ke 0.6
        if (Math.abs(drivePower) < MIN_DRIVE_POWER) {
            drivePower = MIN_DRIVE_POWER * Math.signum(drivePower);
        }

//        Biar tetep lurus dan gk belok miring atau gmn lah misal ntr ditabrak
//        tpi tetep bisa ke degree 0 biar gk salah jalur
        double headingCorrection = 0;
        if (IMUAvaiable) {
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            headingCorrection = turnPID.calculateDegree(heading, currentYaw);
        } else {
//          Ini biar klo ada perbedaan encoder di koreksi
            double encoderDifferent = leftInches - rightInches;
            headingCorrection = turnPID.calculateDegree(heading, encoderDifferent);
        }
        headingCorrection = Range.clip(headingCorrection, -MIN_TURN_POWER, MAX_TURN_POWER);

//        Jadinya ini gk bakal miring gitu klo pun miring bakal langsung di koreksi dan heading nya jadi 0 lagi.
        leftMotor.setPower(drivePower + headingCorrection);
        rightMotor.setPower(drivePower - headingCorrection);

        return false;
    }

    public boolean turnTo(double targetDegrees) {
        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);

        double currentHeading = getHeading();
        double error = turnPID.angleWrapDegree(targetDegrees - currentHeading);

        if (Math.abs(error) < TURN_TOLERANCE_DEGREE) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            turnPID = null;
            return true;
        }
        double turnPower = turnPID.calculateDegree(targetDegrees, currentHeading);
        turnPower = Range.clip(turnPower, -MIN_TURN_POWER, MAX_TURN_POWER);

        if (Math.abs(turnPower) < MIN_TURN_POWER) {
            turnPower = MIN_TURN_POWER * Math.signum(turnPower);
        }

        leftMotor.setPower(turnPower);
        rightMotor.setPower(-turnPower);

        return false;
    }

    public double getHeading() {
        if (IMUAvaiable) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;

//            Jarak antara Roda kanan ke Roda kiri
        final double TRACK_WIDTH_TICKS = 13.0;
        return Math.toDegrees((leftInches - rightInches) / TRACK_WIDTH_TICKS);
    }

    public boolean moveToInDeg(double inches, double sHeading, double turnDegree) {
        straightTo(inches, sHeading);
        turnTo(turnDegree);
        return false;
    }

    public boolean moveToDegIn(double turnDegree, double inches, double sHeading) {
        turnTo(turnDegree);
        straightTo(inches, sHeading);
        return false;
    }

    public boolean resetAndStraightTo(double inches, double sHeading) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnPID = null;
        straightPID = null;

        return straightTo(inches, sHeading);
    }
}
