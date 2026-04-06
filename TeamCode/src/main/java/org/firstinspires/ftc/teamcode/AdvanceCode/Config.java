package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.acmerobotics.dashboard.config.Config
public class Config {
    //    Import Components
    private FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    private PIDFCoefficients pidfCoefficients;
    private HeadingPIDController armPID;
    private HeadingPIDController straightPID;
    private HeadingPIDController turnPID;
    private HeadingPIDController strafePID;
    private HeadingPIDController splinePID;
    //    Drive base Configs
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
    final double WHEEL_DIAMETER_INCHES = 96.0 / 25.4;
    public static double TICKS_PER_REV = 537.6898395722; // Inch per revolution drivebase moto r
    final double INCHES_PER_TICKS = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    private final double SPLINE_TOLERANCE_INCHES = 2.0;
    private final double STRAFE_TOLERANCE_INCHES = 2.0;
    private final double STRAIGHT_TOLERANCE_INCHES = 3.0;
    private final double TURN_TOLERANCE_DEGREE = 1.5;
    private final double MAX_DRIVE_POWER = 0.4;
    private final double MIN_DRIVE_POWER = 0.3;
    private final double MAX_TURN_POWER = 0.35;
    private final double MIN_TURN_POWER = 0.05;
    //    Jarak antara roda kanan & Kiri
    private final double TRACK_WIDTH_INCHES = 14.0;
    // SubSystem Config
    DcMotorEx armMotor = null;
    public static double aKP = 0;
    public static double aKI = 0;
    public static double aKD = 0;
    public static double aKF = 0;
    public static double ticks = 28.0 * 40.0 / 3.0; // Ticks formula (Ticks(28) * Internal ratio * external ratio))


    //    Shooter

    DcMotorEx Shooter = null;
    //    Intake
    DcMotorEx Intake = null;

    //    IMU
    private IMU imu;
    private boolean IMUAvailable = true;

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
    boolean useImu;


    public void initialize(HardwareMap hardwareMap, boolean useTankDrive, boolean useImu, boolean useArm, boolean useShoot, boolean useIntake) {
        this.useTankDrive = useTankDrive;

        this.useImu = useImu;

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
            useMecanum = false;
        } else {
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
            useMecanum = true;
            this.useTankDrive = false;
        }

        if (useImu) {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            imu.initialize(imuParams);
            imu.resetYaw();
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
        double feedforward = Math.cos(angleRad) * aKF; // ini menghasilkan  kencang diawal tpi semakin dekat dengan target dia makin pelan
        double power = pid + feedforward;
        armMotor.setVelocity(power);
    }

    public void Shooter(double targetVel) {
        if (!useShoot || Shooter == null) return;
        Shooter.setVelocity(targetVel);
    }

    public void Intake(double targetPower) {
        if (!useIntake || Intake == null) return;
        Intake.setPower(targetPower);
    }

    public boolean straightTo(double inches, double heading) {
        boolean hasDrive = (leftMotor != null && rightMotor != null);
        if (!hasDrive) return true;

        if (straightPID == null) straightPID = new HeadingPIDController(dKP, dKI, dKD);
        straightPID.kP = dKP;
        straightPID.kI = dKI;
        straightPID.kD = dKP;

        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);
        turnPID.kP = tKP;
        turnPID.kI = tKI;
        turnPID.kD = tKD;


        double currentInches;
        double leftInches = 0, rightInches = 0;

        if (useMecanum && leftBackMotor != null && rightBackMotor != null) {
            leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
            rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
            double leftBackInches = leftBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
            double rightBackInches = rightBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
            currentInches = (leftInches + rightInches + leftBackInches + rightBackInches) / 4.0;
        } else {
            // Tank drive 2 motor
            leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
            rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
            currentInches = (leftInches + rightInches) / 2.0;
        }
        double error = inches - currentInches;

        double drivePower = straightPID.calculateInches(inches, currentInches);
//        klo drivepower nya - di abs sama absDrive jadi nya gk ada - nah terus math.signum tuh tugasnya buat ngembaliin tanda di Min nya jadi bakal -6, 1
//        klo gk pake signum ntr clip hasilnya malah 0.6 bukan - 0.6 jadi yang harusnya mundur malah maju.
        drivePower = Range.clip(drivePower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        //        ini kalau error udah kurang dari 3inches di stop tuh motor nya
        if (Math.abs(error) < STRAIGHT_TOLERANCE_INCHES) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            if (leftBackMotor != null) leftBackMotor.setPower(0);
            if (rightBackMotor != null) rightBackMotor.setPower(0);
            straightPID = null;
            turnPID = null;
            return true;
        }

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
        turnPID.kP = tKP;
        turnPID.kI = tKI;
        turnPID.kD = tKD;

        double currentHeading = getHeading();
        double error = turnPID.angleWrapDegree(targetDegrees - currentHeading);

        if (Math.abs(error) < TURN_TOLERANCE_DEGREE) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            if (leftBackMotor != null) leftBackMotor.setPower(0);
            if (rightBackMotor != null) rightBackMotor.setPower(0);
            resetAllEncoders();
            turnPID.reset();
            turnPID = null;
            return true;
        }
        double turnPower = turnPID.calculateDegree(targetDegrees, currentHeading);
        double absTurn = Range.clip(Math.abs(turnPower), MIN_TURN_POWER, MAX_TURN_POWER);
        turnPower = absTurn * Math.signum(turnPower);
//

//        Disini ada if(leftbackmotor != null ) but yang mechanum jadi jika si init nya itu mechanum true itu jalan, begitu sebaliknya;
        leftMotor.setPower(turnPower);
        if (leftBackMotor != null) leftBackMotor.setPower(turnPower);

        rightMotor.setPower(-turnPower);
        if (rightBackMotor != null) rightBackMotor.setPower(-turnPower);
        return false;
    }

    public boolean strafeTo(double strafeInches) {
        if (strafePID == null) strafePID = new HeadingPIDController(strP, strI, strD);
        strafePID.kP = strP;
        strafePID.kI = strI;
        strafePID.kD = strD;

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
        drivePower = Range.clip(drivePower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        leftMotor.setPower(drivePower);
        leftBackMotor.setPower(-drivePower);

        rightMotor.setPower(-drivePower);
        rightBackMotor.setPower(drivePower);

        return false;
    }

    // spline lurus  lalu belok curve sampai headingnya; pakai cos & sin untuk curve nya;
    public boolean splineTo(double splineInches, double headingRadians, double curveFactor, double tangent) {
        if (splinePID == null) splinePID = new HeadingPIDController(dKP, dKI, dKD);
        splinePID.kP = dKP;
        splinePID.kI = dKI;
        splinePID.kD = dKD;
        if (turnPID == null) turnPID = new HeadingPIDController(tKP, tKI, tKD);
        turnPID.kP = tKP;
        turnPID.kI = tKI;
        turnPID.kD = tKD;


        double leftInches = leftMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightInches = rightMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double leftBackInches = leftBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double rightBackInches = rightBackMotor.getCurrentPosition() * INCHES_PER_TICKS;
        double currentInches = (leftInches + rightInches + leftBackInches + rightBackInches) / 4.0; // Inches / 4.0 karena ada 4 motor


        double error = splineInches - currentInches;

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
        double Progress = Range.clip(currentInches / splineInches, 0.0, 1.0);
//        * curveFactor biar ntr bisa makin tajem atau pu nenggak
        double angle = Math.min(Progress * (Math.PI / 2.0) * curveFactor, Math.PI / 2.0);
//        Tanh tuh mempersmoothhh aja
        double theta = Math.tanh(angle * tangent);
//        cos untuk straightnya. cos dari nilai 1.0 -> 0.0
        double cos = Math.cos(theta * Math.PI / 2); // Math.cos disin waktu diawal kenceng tpi diakhir makin pelan
//        sin untuk curve
//        sin dari nilai 0.0 -> 1.0
        double sin = Math.sin(theta * Math.PI / 2); // Math.sin disini untuk biar power straight nya itu kecil tpi pas mau akhiran kenceng;
//        disini cos nya untuk kenceng diawal berarti straight, dan sin untuk kenceng diakhir yang membuat curve spline yang smooth;

        double splinePower = splinePID.calculateInches(splineInches, currentInches);
        splinePower = Range.clip(splinePower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        double targetHeading = Progress * headingRadians;

//        Heading IMU && Pinpoint
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingError = turnPID.calculateRadians(targetHeading, currentYaw);
        double turnPower = Range.clip(headingError, -MAX_TURN_POWER, MAX_TURN_POWER);
        double drive = splinePower * cos; // dikali cos biar seiring waktu robot maju kecepatanya berkurang
        double turn = turnPower * sin; // dikali cos biar seiring waktu spline robot maju nya makin kenceng

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

        if (Shooter != null) Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (Shooter != null) Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Intake != null) Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (Intake != null) Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // TELEOP CONFIG
    public void tankMotors(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}

