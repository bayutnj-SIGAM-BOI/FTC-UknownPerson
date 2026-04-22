package org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODE.ColorSensor.NormalizeColorSensor;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.DECODE.ableToShootTriangle;
import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
import org.firstinspires.ftc.teamcode.DECODE.Turret.TurretWithPoseEstimate;

@TeleOp

public class TeleopRoadRunner extends OpMode {
    private TankDrive drive;
    private TurretWithPoseEstimate turret;
    private final RobotStatic rC = new RobotStatic();
    private final ableToShootTriangle trig = new ableToShootTriangle();
    NormalizeColorSensor colorSensor;
    NormalizeColorSensor.detectColors detectColors;
    private Servo angleAdjuster, stooperGate;
    private DcMotorEx Shooter, Intake;
    private Pose2d target = RobotStatic.blueAimingTarget;

    enum TriangleState {
        IDLE,
        ANGLE,
        SHOOTING,
        INTAKE,
        OPEN_GATE,
    }

    private ElapsedTime StooperTime = new ElapsedTime();

    @Override
    public void init() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive = new TankDrive(hardwareMap, beginPose);
        turret = new TurretWithPoseEstimate(hardwareMap);

        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");
        stooperGate = hardwareMap.get(Servo.class, "Stooper");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        colorSensor = new NormalizeColorSensor(hardwareMap);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    @Override
    public void loop() {
        drive.localizer.update();
        Pose2d getpose = drive.localizer.getPose();
        double RobotX = getpose.position.x;
        double RobotY = getpose.position.y;
        double Heading = getpose.heading.toDouble();

        double Rotate = -gamepad1.right_stick_x;
        double Forward = gamepad1.left_stick_y;
//        double Strafe = gamepad1.left_stick_x;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(Forward, 0), Rotate));

//        ========== Tracking Poses turret ==========
        if (gamepad1.left_bumper) target = RobotStatic.blueAimingTarget;
        else if (gamepad1.right_bumper) target = RobotStatic.redAimingTarget;
        turret.aimingTurret(target, RobotX, RobotY, Heading);

//        ========== Manually system ==========
        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(RobotStatic.INTAKE_SPEED);
        } else {
            Intake.setPower(0);
        }

//        stooperGate.setPosition(gamepad1.a ? RobotStatic.OPEN_GATE : RobotStatic.CLOSE_GATE);
//        ========== Auto Shooting Triangle ==========
        double distanceTarget = Math.hypot(RobotX - target.position.x, RobotY - target.position.y);

        detectColors = colorSensor.getDetectedColor(telemetry);
        boolean PurpleColor = detectColors == NormalizeColorSensor.detectColors.PURPLE;
        boolean GreenColor = detectColors == NormalizeColorSensor.detectColors.GREEN;
        boolean UnknownColor = detectColors == NormalizeColorSensor.detectColors.UNKNOWN;

        updateShooterSub(RobotX, RobotY, distanceTarget, PurpleColor, GreenColor, UnknownColor);

        telemetry.addData("Pose", getpose);
        telemetry.addData("X", RobotX);
        telemetry.addData("Y", RobotY);
        telemetry.addData("Heading", Heading);
        telemetry.update();
    }

    TriangleState currentState = TriangleState.IDLE;

    private void updateShooterSub(double RobotX, double RobotY, double distanceTarget, boolean PurpleColor, boolean GreenColor, boolean Unknown) {
       if (trig.ableToShoot(RobotX, RobotY) && (PurpleColor || GreenColor) && !Unknown) {
           switch (currentState) {
               case IDLE:
                   currentState = TriangleState.ANGLE;

               case ANGLE:
                   angleAdjuster.setPosition(rC.AngleAdjuster(distanceTarget));
                   currentState = TriangleState.SHOOTING;

               case SHOOTING:
                   Shooter.setVelocity(rC.EveryWhereShooInterpolation(distanceTarget));
                   if (Math.abs(Shooter.getVelocity() - rC.EveryWhereShooInterpolation(distanceTarget)) < 50.0) {
                       currentState = TriangleState.INTAKE;
                   }
                   break;

               case INTAKE:
                   Intake.setPower(RobotStatic.INTAKE_SPEED);
                   currentState = TriangleState.OPEN_GATE;

               case OPEN_GATE:
                   stooperGate.setPosition(RobotStatic.OPEN_GATE);
                   if (StooperTime.seconds() > 1.2) {
                       stooperGate.setPosition(RobotStatic.CLOSE_GATE);
                   }
                   break;
           }
       } else {
           currentState = TriangleState.IDLE;
           Shooter.setVelocity(0);
           Intake.setPower(0);
           stooperGate.setPosition(RobotStatic.CLOSE_GATE);
       }
    }
}
