package org.firstinspires.ftc.teamcode.DECODE.Basic;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
import org.firstinspires.ftc.teamcode.tuning.SpeedTunning;

@TeleOp(name = "EXHIBITION TELEOP")
public class FieldOrientationMecanum extends OpMode {
    DcMotor leftFront, rearLeft, rightFront, rearRight;
    DcMotor turret, Intake;
    DcMotorEx flyWheel;
    IMU imu;

    private final double encRev = 0;
    private final double grat = 5.0;
    private final double rotDeg = (encRev * grat) / 360.0;
    private final double rotLimit = 135.0;

    private final double slowMode = 0.45;

    private enum IntakeState {
        STOP,
        INTAKE_FORWARD,
        INTAKE_REVERSED
    }
    private enum ShooterState {
        STOP,
        LAUNCH
    }
    IntakeState currentState = IntakeState.STOP;
    ShooterState scState = ShooterState.STOP;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rightFront =  hardwareMap.get(DcMotor.class, "rightFront");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flywheelPID =  new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPID);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        boolean slowBtn = gamepad1.left_bumper;

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
        double rotY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);
        if (slowBtn) {
            rotX *= slowMode;
            rotY *= slowMode;
            rx *= slowMode;
        }
        double clamp = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        leftFront.setPower((rotY + rotX + rx) / clamp);
        rearLeft.setPower((rotY - rotX + rx) / clamp);
        rightFront.setPower((rotY - rotX - rx) / clamp);
        rearRight.setPower((rotY + rotX - rx) / clamp);

        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean Y = gamepad1.y;
        if (lt > 0.1 && currentState != IntakeState.STOP) {currentState = IntakeState.INTAKE_FORWARD;}
        else if (rt > 0.1 && currentState != IntakeState.STOP) { currentState = IntakeState.INTAKE_REVERSED;}
        else {currentState = IntakeState.STOP;}
        switch (currentState) {
            case INTAKE_FORWARD:
                Intake.setPower(RobotStatic.INTAKE_SPEED);
                break;
            case INTAKE_REVERSED:
                Intake.setPower(-RobotStatic.INTAKE_SPEED);
                break;
            case STOP:
            default:
                Intake.setPower(0);
                currentState = IntakeState.STOP;
                break;
        }
        if (Y && scState != ShooterState.STOP) {scState = ShooterState.LAUNCH;}
        else {scState = ShooterState.STOP;}
       switch (scState) {
           case LAUNCH:
               flyWheel.setVelocity(1800);
               break;
           case STOP:
           default:
               flyWheel.setVelocity(0);
               scState = ShooterState.STOP;
               break;
       }

        manualTurret(gamepad2.left_stick_x);

        telemetry.addLine("ROBOT");
        telemetry.addData("R Heading", Math.toDegrees(robotHeading));
        telemetry.addLine("TURRET SYSTEM");
        telemetry.addData("C Degree", turret.getCurrentPosition() / rotDeg);
        telemetry.addLine("STATE");
        telemetry.addData("Intake STATE", currentState);
        telemetry.addData("Shooter STATE", scState);
        telemetry.update();
    }

    private void manualTurret(double x) {
        double cD = turret.getCurrentPosition() / rotDeg;

        if (cD > rotLimit && x > 0) {
            turret.setPower(0);
        } else if (cD < -rotLimit && x < 0) {
            turret.setPower(0);
        } else if (Math.abs(x) > x) {
            turret.setPower(x * 0.6);
        } else {
            turret.setPower(0);
        }
    }
}
