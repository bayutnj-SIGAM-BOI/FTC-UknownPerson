package org.firstinspires.ftc.teamcode.DECODE.Basic;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DECODE.PIDControl;
import org.firstinspires.ftc.teamcode.DECODE.robotConfiguration;
import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.TurretSub;

@TeleOp(name = "EXHIBITION TELEOP", group = "EXHIBITION")
public class ExhibitionMecanumTeleOp extends OpMode {
    private PIDControl pidControl = new PIDControl(1, 0, 0.0008);
    private GoBildaPinpointDriver odo;
    private TurretSub turret;
    private robotConfiguration rc = new robotConfiguration();
    private DcMotor leftFront, rearLeft, rightFront, rearRight;
    private DcMotor Intake;
    private DcMotorEx flyWheel;
    private Servo hoodAngle;

    private final double slowMode = 0.45;

    private enum IntakeState {
        STOP, INTAKE_FORWARD, INTAKE_REVERSED
    }

    private enum ShooterState {
        STOP, LAUNCH
    }

    IntakeState currentState = IntakeState.STOP;
    ShooterState scState = ShooterState.STOP;
    Pose2d target = robotConfiguration.blueAimingTarget;
    private boolean isRedAlliance = false, isBlueAlliance = true;

    private final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private final double SNAP_0 = 0;
    private final double SNAP_90 = 90;
    private final double SNAP_180 = 180;
    private final double SNAP_REVERSED_180 = -180;
    private boolean isAutoHeading = true;
    private boolean targetLocking = false;
    private double targetSnap = 0;

    @Override
    public void init() {
        turret = new TurretSub(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flywheelPID = new PIDFCoefficients(200, 0, 0, 13.1000);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPID);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
        hoodAngle.setDirection(Servo.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-7.5, 8.5, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.setPosition(START_POSE);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            target = robotConfiguration.blueAimingTarget;
            isBlueAlliance = true;
            isRedAlliance = false;
            telemetry.addData("ALLIANCE TARGET", "BLUE");
        } else if (gamepad1.b) {
            target = robotConfiguration.redAimingTarget;
            isRedAlliance = true;
            isBlueAlliance = false;
            telemetry.addData("ALLIANCE TARGET", "RED");
        }
        telemetry.addData("ALLIANCE", isBlueAlliance ? "BLUE" : "RED");
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();
        Pose2D pos = odo.getPosition();
        double yVel = odo.getVelY(DistanceUnit.INCH);
        double xVel = odo.getVelX(DistanceUnit.INCH);
        double OffsetX = odo.getXOffset(DistanceUnit.CM);
        double OffsetY = odo.getYOffset(DistanceUnit.CM);
        double dx = pos.getX(DistanceUnit.INCH);
        double dy = pos.getY(DistanceUnit.INCH);
        double heading = pos.getHeading(AngleUnit.RADIANS);
        double VelX = odo.getVelX(DistanceUnit.INCH);
        double VelY = odo.getVelY(DistanceUnit.INCH);
        double dist = turret.getDistanceTarget(dx, dy, heading, target);

        turret.aimingTurret(target, dx, dy, heading, xVel, yVel);
        hoodAngle.setPosition(rc.hoodAngle(dist));

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.dpad_right) {
            targetSnap = SNAP_0;
            isAutoHeading = true;
        }
        if (gamepad1.dpad_left) {
            targetSnap = SNAP_90;
            isAutoHeading = true;
        }
        if (gamepad1.dpad_up) {
            targetSnap = SNAP_180;
            isAutoHeading = true;
        }
        if (gamepad1.dpad_down) {
            targetSnap = SNAP_REVERSED_180;
            isAutoHeading = true;
        }

        if (Math.abs(rx) > 0.5) {
            isAutoHeading = false;
            targetLocking = false;
        }

        if (gamepad1.right_bumper) {
            double calculateTargetAngle = Math.atan2(dy - target.position.y, dx - target.position.x);
            targetSnap = pidControl.angleWrapRadians(Math.toDegrees(calculateTargetAngle));
            targetLocking = true;
            isAutoHeading = true;
        } else {
            targetLocking = false;
            isAutoHeading = false;
        }

        if (isAutoHeading) {
            double eH = pidControl.angleWrapRadians(Math.toRadians(targetSnap) - heading);
            if (Math.abs(eH) < Math.toRadians(1.5)) {
                isAutoHeading = false;
                rx = 0;
            } else {
                rx = pidControl.calculateRadians(Math.toRadians(targetSnap), heading);
            }
        }

        double clamp = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower((y + x + rx) / clamp);
        rearLeft.setPower((y - x + rx) / clamp);
        rightFront.setPower((y - x - rx) / clamp);
        rearRight.setPower((y + x - rx) / clamp);

        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean Y = gamepad1.y;
        if (lt > 0.1) {
            currentState = IntakeState.INTAKE_FORWARD;
        } else if (rt > 0.1) {
            currentState = IntakeState.INTAKE_REVERSED;
        } else {
            currentState = IntakeState.STOP;
        }
        switch (currentState) {
            case INTAKE_FORWARD:
                Intake.setPower(robotConfiguration.INTAKE_SPEED);
                break;
            case INTAKE_REVERSED:
                Intake.setPower(-robotConfiguration.INTAKE_SPEED);
                break;
            default:
                Intake.setPower(0);
                break;
        }
        if (Y) {
            scState = ShooterState.LAUNCH;
        } else {
            scState = ShooterState.STOP;
        }
        switch (scState) {
            case LAUNCH:
                flyWheel.setVelocity(rc.flywheelSpeed(dist));
                break;
            default:
                flyWheel.setVelocity(0);
                break;
        }

        if (gamepad1.back) {
            odo.resetPosAndIMU();
            telemetry.addData("ODO", "RESET");
        }

        telemetry.addLine("ROBOT");
        telemetry.addData("ROBOT X", dx);
        telemetry.addData("ROBOT Y", dy);
        telemetry.addData("VEL X", VelX);
        telemetry.addData("VEL Y", VelY);
        telemetry.addData("CURRENT POS", pos);
        telemetry.addData("OFFSET X", OffsetX);
        telemetry.addData("OFFSET Y", OffsetY);
        telemetry.addData("R Heading", Math.toDegrees(heading));
        telemetry.addData("TARGET LOCKING", targetLocking ? "Locked" : "Not Lock");
        telemetry.addData("Distance TARGET", dist);

        telemetry.addLine("TURRET");
        telemetry.addData("Is Turret Aimed", turret.isAimed() ? "Aimed" : "Not Aimed");
        telemetry.addData("HOOD POSITION", hoodAngle.getPosition());
        telemetry.addData("Shotter TARGET VEL ERROR", flyWheel.getVelocity() - rc.flywheelSpeed(dist));
        telemetry.addData("TARGET Shooter VELOCITY", rc.flywheelSpeed(dist));
        telemetry.addData("CURRENT DEG", turret.turretDeg());

        telemetry.addLine("STATE");
        telemetry.addData("Intake STATE", currentState);
        telemetry.addData("Shooter STATE", scState);
        telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
        telemetry.update();
    }

//
}
