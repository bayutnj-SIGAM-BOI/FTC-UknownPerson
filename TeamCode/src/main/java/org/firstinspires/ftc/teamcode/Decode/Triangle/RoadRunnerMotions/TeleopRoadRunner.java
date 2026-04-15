package org.firstinspires.ftc.teamcode.Decode.Triangle.RoadRunnerMotions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.Decode.Triangle.ableToShootTriangle;
import org.firstinspires.ftc.teamcode.Decode.Triangle.RobotConstant;
import org.firstinspires.ftc.teamcode.Decode.Triangle.Turret.TurretWithPoseEstimate;

@TeleOp

public class TeleopRoadRunner extends OpMode {
    private Servo angleAdjuster, stooperGate;
    private TankDrive drive;
    private DcMotorEx Shooter, Intake;
    private TurretWithPoseEstimate turret;
    private final RobotConstant rC = new RobotConstant();
    private final ableToShootTriangle trig = new ableToShootTriangle();
    private Pose2d target = RobotConstant.blueAimingTarget;

    @Override
    public void init() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive = new TankDrive(hardwareMap, beginPose);
        turret = new TurretWithPoseEstimate(hardwareMap);

        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");
        stooperGate = hardwareMap.get(Servo.class, "Stooper");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double distanceTarget = Math.hypot(RobotX - target.position.x, RobotY - target.position.y);

        double x = -gamepad1.right_stick_x;
        double y = gamepad1.left_stick_y;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, 0), x));

        stooperGate.setPosition(gamepad1.a ? RobotConstant.OPEN_GATE : RobotConstant.CLOSE_GATE);

        if (gamepad1.left_bumper) target = RobotConstant.blueAimingTarget;
        else if (gamepad1.right_bumper) target = RobotConstant.redAimingTarget;
        turret.aimingTurret(target, RobotX, RobotY, Heading);

        if (trig.ableToShoot(RobotX, RobotY)) {
            stooperGate.setPosition(RobotConstant.OPEN_GATE);
            angleAdjuster.setPosition(rC.AngleAdjuster(distanceTarget));

            if (Shooter.getVelocity() > 0 || Intake.getCurrentPosition() > 0) {
                Shooter.setVelocity(rC.EveryWhereShooInterpolation(distanceTarget));
                Intake.setPower(RobotConstant.INTAKE_SPEED);
            }
        } else {
            stooperGate.setPosition(RobotConstant.CLOSE_GATE);
            Shooter.setVelocity(0);
            Intake.setPower(0);
        }

        telemetry.addData("Pose", getpose);
        telemetry.addData("X", RobotX);
        telemetry.addData("Y", RobotY);
        telemetry.addData("Heading", Heading);
        telemetry.update();
    }
}
