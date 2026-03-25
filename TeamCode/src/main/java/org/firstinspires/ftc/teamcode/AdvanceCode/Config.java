package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
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
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    PIDFCoefficients pidfCoefficients;
    HeadingPIDController armPID;
    HeadingPIDController straightPID;
    HeadingPIDController turnPID;
    HeadingPIDController strafePID;
    HeadingPIDController splinePID;
    //    Drive base Config
    DcMotor leftMotor, rightMotor = null;
    DcMotor leftBackMotor, rightBackMotor = null;

    public static double dKP = 0;
    public static double dKI = 0;
    public static double dKD = 0;

    //    strafePID
    public static double strP = 0;
    public static double strI = 0;
    public static double strD = 0;
    //    Turn PID
    public static double tKP = 0.008;
    public static double tKI = 0.000013;
    public static double tKD = 0.005;

    public static double sKP = 12.65;
    public static double sKI = 0;
    public static double sKD = 0;
    public static double sKF = 200;
    //    Drive Constants
    final double WHEEL_DIAMETER_INCHES = 4.33071;
    public static double TICKS_PER_REV = 118.09090909091;
    final double INCHES_PER_TICKS = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    final double SPLINE_TOLERANCE_INCHES = 2.0;
    final double STRAFE_TOLERANCE_INCHES = 2.0;
    final double STRAIGHT_TOLERANCE_INCHES = 2.0;
    final double TURN_TOLERANCE_DEGREE = 1.5;
    final double MAX_DRIVE_POWER = 0.4;
    final double MIN_DRIVE_POWER = 0.3;
    final double MAX_TURN_POWER = 0.35;
    final double MIN_TURN_POWER = 0.05;
    //    Jarak antara roda kanan & Kiri
    final double TRACK_WIDTH_INCHES = 13.0;
    // SubSystem Config
    DcMotorEx armMotor = null;
    public static double aKP = 0;
    public static double aKI = 0;
    public static double aKD = 0;
    public static double aKF = 0;
    public static double ticks = 28.0 * 40.0 / 3.0; // Ticks formula (Ticks(28) * Internal ratio * external ratio))

    //    Shooter

    DcMotorEx Shooter = null;
    boolean allowShoot = false;
    //    Intake
    DcMotorEx Intake = null;
    boolean allowIntake = false;

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

    //    use Systems
    boolean useMecanum, useTankDrive;
    boolean useArm, useShoot, useIntake;

    public void initialize(HardwareMap hardwareMap, boolean useMecanum, boolean useTankDrive, boolean useArm, boolean useShoot, boolean useIntake) {
        this.useMecanum = useMecanum;
        this.useTankDrive = useTankDrive;

        this.useArm = useArm;
        this.useShoot = useShoot;
        this.useIntake = useIntake;

//        Drive base initialize
        if (useTankDrive) {
            leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
            rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (useMecanum) {
            leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
            rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
            leftBackMotor = hardwareMap.get(DcMotor.class, "leftbackMotor");
            rightBackMotor = hardwareMap.get(DcMotor.class, "rightbackMotor");

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // SubSystem initialize
        if (useArm) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (useShoot) {
            Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
            Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pidfCoefficients = new PIDFCoefficients(sKP, sKI, sKD, sKF);
            Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }

        if (useIntake) {
            Intake = hardwareMap.get(DcMotorEx.class, "Intake");
            Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

//        IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParams);
        imu.resetYaw();
        IMUAvailable = true;

    }

    public void armTo(int targetDegree) {
        if (!useArm || armMotor == null) return;
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
        if (!useShoot || Shooter == null) return true;
        if (allowShoot) {
            Shooter.setVelocity(targetVel);
        } else {
            Shooter.setVelocity(0);
        }

        return false;
    }

    public boolean Intake(double targetTicks) {
        if (!useIntake || Intake == null) return true;
        if (allowIntake) {
            Intake.setVelocity(targetTicks * 60);
        } else {
            Intake.setVelocity(0);
        }
        return false;
    }

    public boolean straightTo(double inches, double heading) {
        boolean hasDrive = (leftMotor != null && rightMotor != null);
        if (!hasDrive) return true;

        if (straightPID == null) straightPID = new HeadingPIDController(dKP, dKI, dKD);
        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);


        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double currentInches;
        if (useMecanum && leftBackMotor != null && rightBackMotor != null) {
            double leftBackInches = leftBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
            double rightBackInches = rightBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
            currentInches = (leftInches + rightInches + leftBackInches + rightBackInches) / 4.0;
        } else {
            // Tank drive 2 motor
            currentInches = (leftInches + rightInches) / 2.0;
        }
        double error = inches - currentInches;

//        ini kalau error udah kurang dari 0.5inches di stop tuh motor nya
        if (Math.abs(error) < STRAIGHT_TOLERANCE_INCHES) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            resetAllEncoders();
            straightPID.reset();
            turnPID.reset();
            straightPID = null;
            turnPID = null;
            return true;
        }
        double drivePower = straightPID.calculateInches(inches, currentInches);
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
            double encoderDiff = (leftInches - rightInches);
            headingCorrection = turnPID.calculateDegree(heading, encoderDiff);
        }
        headingCorrection = Range.clip(headingCorrection, -MAX_TURN_POWER, MAX_TURN_POWER);

//        Jadinya ini gk bakal miring gitu klo pun miring bakal langsung di koreksi dan heading nya jadi 0 lagi.
        leftMotor.setPower(drivePower + headingCorrection);
        rightMotor.setPower(drivePower - headingCorrection);
        if (useMecanum && leftBackMotor != null && rightBackMotor != null) {
            leftBackMotor.setPower(drivePower + headingCorrection);
            rightBackMotor.setPower(drivePower - headingCorrection);
        }

        return false;
    }

    public boolean turnTo(double targetDegrees) {
        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);

        double currentHeading = getHeading();
        double error = turnPID.angleWrapDegree(targetDegrees - currentHeading);

        if (Math.abs(error) < TURN_TOLERANCE_DEGREE) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            resetAllEncoders();
            turnPID.reset();
            turnPID = null;
            return true;
        }
        double turnPower = turnPID.calculateDegree(targetDegrees, currentHeading);
        double absTurn = Range.clip(Math.abs(turnPower), MIN_TURN_POWER, MAX_TURN_POWER);
        turnPower = absTurn * Math.signum(turnPower);
//        double turnPower = (error * tKP);  // Sementara tanpa I dan D dulu!
//        turnPower = Range.clip(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

//        Disini ada if(leftbackmotor != null ) but yang mechanum jadi jika si init nya itu mechanum true itu jalan, begitu sebaliknya;
        leftMotor.setPower(turnPower);
        if (leftBackMotor != null) leftBackMotor.setPower(turnPower);

        rightMotor.setPower(-turnPower);
        if (rightBackMotor != null) rightBackMotor.setPower(-turnPower);
        return false;
    }

    public boolean strafeTo(double strafeInches) {
        if (strafePID == null) strafePID = new HeadingPIDController(strP, strI, strD);

        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double leftBackInches = leftBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightBackInches = rightBackMotor.getCurrentPosition() * INCHES_PER_TICKS;

        double currentInches = (leftInches - rightInches - leftBackInches + rightBackInches) / 4.0;
        double error = strafeInches - currentInches;

        if (Math.abs(error) < STRAFE_TOLERANCE_INCHES) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            resetAllEncoders();
            strafePID.reset();
            strafePID = null;
            return true;
        }
        double drivePower = strafePID.calculateInches(strafeInches, currentInches);
        double absPower = Range.clip(Math.abs(drivePower), MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        drivePower = absPower * Math.signum(drivePower);

        leftMotor.setPower(drivePower);
        leftBackMotor.setPower(-drivePower);

        rightMotor.setPower(-drivePower);
        rightBackMotor.setPower(drivePower);

        return false;
    }

    // spline lurus  lalu belok curve sampai headingnya; pakai cos & sin untuk curve nya;
    public boolean splineTo(double splineInches, double headingRadians, double curveFactor, double tangent) {
        if (splinePID == null) splinePID = new HeadingPIDController(dKP, dKI, dKD);
        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);

        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double leftBackInches = leftBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightBackInches = rightBackMotor.getCurrentPosition() * INCHES_PER_TICKS;

        double currentInches = (leftInches + rightInches + leftBackInches + rightBackInches) / 4.0; // Inches / 4.0 karena ada 4 motor
        double error = currentInches - splineInches;

        if (Math.abs(error) < SPLINE_TOLERANCE_INCHES) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            resetAllEncoders();
            splinePID.reset();
            turnPID.reset();
            splinePID = null;
            turnPID = null;
            return true;
        }

//        progress 0 -> 1;
        double t = Math.min(Math.max(currentInches / splineInches, 0.0), 1.0);
//        * curveFactor biar ntr bisa makin tajem atau pu nenggak
        double angle = Math.min(t * (Math.PI / 2.0) * curveFactor, Math.PI / 2.0);
//        cos untuk straightnya. cos dari nilai 1.0 -> 0.0
        double cos = Math.cos(angle); // Math.cos disin waktu diawal kenceng tpi diakhir makin pelan
//        sin untuk curve
//        sin dari nilai 0.0 -> 1.0
        double sin = Math.sin(angle); // Math.sin disini untuk biar power curve nya itu kecil tpi pas mau akhiran kenceng;
//        disini cos nya untuk kenceng diawal berarti straight, dan sin untuk kenceng diakhir yang membuat curve spline yang smooth;
        double theta = Math.tanh(angle);

        double splinePower = splinePID.calculateInches(splineInches, currentInches);
        double absPower = Range.clip(Math.abs(splinePower), MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        splinePower = absPower * Math.signum(splinePower);

        double tHeading = t * headingRadians;

//        Heading IMU
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingError = turnPID.calculateRadians(tHeading, currentYaw);
        double turnPower = Range.clip(headingError, -MAX_TURN_POWER, MAX_TURN_POWER);

        double drive = splinePower * cos;
        double turn = turnPower * sin;

        leftMotor.setPower(drive + turn);
        rightMotor.setPower(drive - turn);
        leftBackMotor.setPower(drive + turn);
        rightBackMotor.setPower(drive - turn);
        return false;
    }

    public double getHeading() {
        if (IMUAvailable) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;

        if (useMecanum && leftBackMotor != null && rightBackMotor != null) {
            double leftBackInches = leftBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
            double rightBackInches = rightBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
            return Math.toDegrees((leftInches - rightInches + leftBackInches - rightBackInches) / (2.0 * TRACK_WIDTH_INCHES));
        }

        return Math.toDegrees((leftInches - rightInches) / TRACK_WIDTH_INCHES);
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
        if (leftBackMotor != null) leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightBackMotor != null) rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (leftBackMotor != null) leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (rightBackMotor != null) rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        straightPID = null;

        return straightTo(inches, sHeading);
    }

    public boolean resetAndTurnTo(double turnDegree) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (leftBackMotor != null) leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightBackMotor != null) rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (leftBackMotor != null) leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (rightBackMotor != null) rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnPID = null;

        return turnTo(turnDegree);
    }

    public void resetAllEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (leftBackMotor != null) leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightBackMotor != null) rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (leftBackMotor != null) leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (rightBackMotor != null) rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
