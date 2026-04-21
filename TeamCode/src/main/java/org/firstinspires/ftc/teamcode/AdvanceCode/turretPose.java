package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.opencv.core.Mat;

@TeleOp
@Disabled
@com.acmerobotics.dashboard.config.Config
public class turretPose extends OpMode {

    TankDrive drive;
    //    GoBildaPinpointDriver odo;
    //    Ini bisa diambil dari MeepMeep
    private final Pose2d blueAimingTarget = new Pose2d(-57.1, -55.3, 0);

    ElapsedTime spinTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private final double integralLimit = 15.0;
    Config r = new Config();
    double leftPower = 0.0;
    double rightPower = 0.0;

    DcMotorEx spinTurret;
    private final double TICKS_PER_REV = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    private final double GearRatio = 5.0;
    private final double OutputSpeed = TICKS_PER_REV * GearRatio;
    private final double TicksPerDegree = OutputSpeed / 360.0;
    private final double turretLimitDeg = 135.0;
    public static double kP = 0.05;
    public static double kI = 0.001;

    public static double kD = 0.003;

    public static double kF = 0.0;


    @Override
    public void init() {

        Pose2d beginPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, beginPose);


        r.initialize(hardwareMap, true, false, false, true);

        spinTurret = hardwareMap.get(DcMotorEx.class, "spinTurret");
        spinTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(-10, 12, DistanceUnit.CM);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        spinTimer.reset();
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        double RobotX = pose.position.x;
        double RobotY = pose.position.y;
        double Heading = pose.heading.toDouble();
//        odo.update();

//        double RobotX = odo.getPosX(DistanceUnit.INCH);
//        double RobotY = odo.getPosY(DistanceUnit.INCH);
//        double Heading = odo.getHeading(AngleUnit.DEGREES);
        aimingTurret(RobotX, RobotY, Heading);

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.right_stick_x;

        this.leftPower = Range.clip(y - x, -1.0, 1.0);
        this.rightPower = Range.clip(y + x, -1.0, 1.0);

        r.tankMotors(leftPower, rightPower);

        double turretDeg = spinTurret.getCurrentPosition() / TicksPerDegree;

        telemetry.addData("Robot X", RobotX);
        telemetry.addData("Robot Y", RobotY);
        telemetry.addData("Heading", Heading);
        telemetry.addData("Turret Deg", turretDeg);
        telemetry.update();
    }

    private void aimingTurret(double RobotX, double RobotY, double robotHeading) {
        double x = blueAimingTarget.position.x - RobotX;
        double y = blueAimingTarget.position.y - RobotY;

        double angle = Math.atan2(y, x);
        double targetAngle = angleWrapDegree(Math.toDegrees(angle) - robotHeading);

        targetAngle = Range.clip(targetAngle, -turretLimitDeg, turretLimitDeg);
        double currentDeg = spinTurret.getCurrentPosition() / TicksPerDegree;


        double error = targetAngle - currentDeg;
        spinTurret.setPower(Range.clip(calculatePID(error), -1, 1));
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) < 1.0) {
            integralSum = 0.0;
            lastError = 0.0;
            return 0;
        }

        if (Math.abs(error) < 10.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD) + kF;
        return output;
    }

    public double angleWrapDegree(double degree) {
        while (degree > 180) {
            degree -= 360;
        }
        while (degree < -180) {
            degree += 360;
        }
        return degree;
    }
}
