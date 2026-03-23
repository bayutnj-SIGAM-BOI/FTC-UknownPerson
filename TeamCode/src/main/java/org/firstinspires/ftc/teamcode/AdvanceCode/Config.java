package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.acmerobotics.dashboard.config.Config
public class Config {
    //    Import Components
    PIDFCoefficients pidfCoefficients;
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

    //    Shooter
    public static double sKP = 0;
    public static double sKI = 0;
    public static double sKD = 0;
    public static double sKF = 0;
    DcMotorEx Shooter = null;
    boolean allowShoot;

    //    IMU
    IMU imu;
    boolean IMUAvailable = false;

    //    State
    enum State {
        STRAIGHT,
        TURN,
        DONE,
    }

    State MoveToInDeg = State.STRAIGHT;
    State MoveToDegIn = State.TURN;

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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    public boolean Shooter(double targetVel) {
        pidfCoefficients = new PIDFCoefficients(sKP, sKI, sKD, sKF);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        if (allowShoot) {
            Shooter.setVelocity(targetVel);
        } else {
            Shooter.setVelocity(0);
        }

        return false;
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
//        klo drivepower nya - di abs sama absDrive jadi nya gk ada - nah terus math.signum tuh tugasnya buat ngembaliin tanda di Min nya jadi bakal -6, 1
//        klo gk pake signum ntr clip hasilnya malah 0.6 bukan - 0.6 jadi yang harusnya mundur malah maju.
        double absDrive = Range.clip(Math.abs(drivePower), MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        drivePower = absDrive * Math.signum(drivePower);

//        Biar tetep lurus dan gk belok miring atau gmn lah misal ntr ditabrak
//        tpi tetep bisa ke degree 0 biar gk salah jalur
        double headingCorrection = 0;
        if (IMUAvailable) {
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            headingCorrection = turnPID.calculateDegree(heading, currentYaw);
        } else {
//          Ini biar klo ada perbedaan encoder di koreksi
            double encoderDifferent = leftInches - rightInches;
            headingCorrection = turnPID.calculateDegree(heading, encoderDifferent);
        }
        double absHeading = Range.clip(Math.abs(headingCorrection), MIN_TURN_POWER, MAX_TURN_POWER);
        headingCorrection = absHeading * Math.signum(headingCorrection);

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
        double absTurn = Range.clip(Math.abs(turnPower), MIN_TURN_POWER, MAX_TURN_POWER);
        turnPower = absTurn * Math.signum(turnPower);

        leftMotor.setPower(turnPower);
        rightMotor.setPower(-turnPower);

        return false;
    }

    public double getHeading() {
        if (IMUAvailable) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;

//            Jarak antara Roda kanan ke Roda kiri
        final double TRACK_WIDTH_TICKS = 13.0;
        return Math.toDegrees((leftInches - rightInches) / TRACK_WIDTH_TICKS);
    }

    public boolean moveToInDeg(double inches, double sHeading, double turnDegree) {
        switch (MoveToInDeg) {
            case STRAIGHT:
                if (straightTo(inches, sHeading)) {
                    MoveToInDeg = State.TURN;
                }
                break;

            case TURN:
                if (turnTo(turnDegree)) {
                    MoveToInDeg = State.DONE;
                }
                break;

            case DONE:
                MoveToInDeg = State.STRAIGHT;
                return true;
        }
        return false;
    }

    public boolean moveToDegIn(double turnDegree, double inches, double sHeading) {

        switch (MoveToDegIn) {
            case TURN:
                if (turnTo(turnDegree)) {
                    MoveToDegIn = State.STRAIGHT;
                }
                break;

            case STRAIGHT:
                if (straightTo(inches, sHeading)) {
                    MoveToDegIn = State.DONE;
                }
                break;

            case DONE:
                MoveToDegIn = State.TURN;
                return true;
        }
        return false;
    }

    public boolean resetAndStraightTo(double inches, double sHeading) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        straightPID = null;

        return straightTo(inches, sHeading);
    }

    public boolean resetAndTurnTo(double turnDegree) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnPID = null;

        return turnTo(turnDegree);
    }

    public void resetAllEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
