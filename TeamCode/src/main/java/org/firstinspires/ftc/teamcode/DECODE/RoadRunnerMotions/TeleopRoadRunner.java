package org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Enter;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    NormalizeColorSensor backTop, backDown, frontSide;
    NormalizeColorSensor.detectColors backTopColor, backDownColor, frontSideColor;
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

    private TriangleState currentState = TriangleState.IDLE;
    private final ElapsedTime StooperTime = new ElapsedTime();

    @Override
    public void init() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive = new TankDrive(hardwareMap, beginPose);
        turret = new TurretWithPoseEstimate(hardwareMap);

        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");
        stooperGate = hardwareMap.get(Servo.class, "Stooper");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        backTop = new NormalizeColorSensor(hardwareMap, "backTop");
        backDown = new NormalizeColorSensor(hardwareMap, "backDown");
        frontSide = new NormalizeColorSensor(hardwareMap, "frontSide");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        stooperGate.setPosition(RobotStatic.CLOSE_GATE);
    }

    @Override
    public void start() {
        StooperTime.reset();
        currentState = TriangleState.IDLE;
    }

    @Override
    public void loop() {
        drive.localizer.update();
        Pose2d getPose = drive.localizer.getPose();
        double RobotX = getPose.position.x;
        double RobotY = getPose.position.y;
        double Heading = getPose.heading.toDouble();

        double distanceTarget = Math.hypot(RobotX - target.position.x, RobotY - target.position.y);

        double slowModeSpeed = (gamepad1.right_trigger > 0.4) ? 0.4 : 1.0;
        double Rotate = -gamepad1.right_stick_x;
        double Forward = gamepad1.left_stick_y;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(Forward * slowModeSpeed, 0), Rotate * slowModeSpeed));

//        ========== Tracking Poses turret ==========
        if (gamepad1.left_bumper) {
                target = RobotStatic.blueAimingTarget;
                gamepad1.setLedColor(0, 0, 1, 100);
            } else if (gamepad1.right_bumper) {
                target = RobotStatic.redAimingTarget;
                gamepad1.setLedColor(1, 0, 0, 100);
            }
        turret.aimingTurret(target, RobotX, RobotY, Heading);

        if (turret.isAimed()) {
            gamepad1.setLedColor(0, 1, 0, -1);
        }

//        ========== Manually system ==========
        if (gamepad1.left_trigger > 0.1) { Intake.setPower(RobotStatic.INTAKE_SPEED);
            } else { Intake.setPower(0); }
        if (gamepad1.a) { Intake.setPower(0); }

//        Color sensor Declaration
        backTopColor = backTop.getDetectedColor(telemetry);
        backDownColor = backDown.getDetectedColor(telemetry);
        frontSideColor = frontSide.getDetectedColor(telemetry);

        boolean PurpleColor = backTopColor == NormalizeColorSensor.detectColors.PURPLE ||
                    backDownColor == NormalizeColorSensor.detectColors.PURPLE ||
                    frontSideColor == NormalizeColorSensor.detectColors.PURPLE;

        boolean GreenColor = backTopColor == NormalizeColorSensor.detectColors.GREEN ||
                    backDownColor == NormalizeColorSensor.detectColors.GREEN ||
                    frontSideColor == NormalizeColorSensor.detectColors.GREEN;

        boolean UnknownColor = backTopColor == NormalizeColorSensor.detectColors.UNKNOWN ||
                    backDownColor == NormalizeColorSensor.detectColors.UNKNOWN ||
                    frontSideColor == NormalizeColorSensor.detectColors.UNKNOWN;

//        Auto Intake if no artifact
        if (!PurpleColor && !GreenColor && !UnknownColor) { Intake.setPower(1);}
//        Rumble when Gate is Open
        if (currentState == TriangleState.OPEN_GATE) { gamepad1.rumble(1.0, 1.0, 300);}

//        State Machine logic auto shooting when the robot is on the shooting Zone
        updateShooterSub(RobotX, RobotY, distanceTarget, PurpleColor, GreenColor, UnknownColor);

//        Helper driver show able to shoot or not
        if (!trig.ableToShoot(RobotX, RobotY)) { gamepad1.setLedColor(1, 0, 0, -1);
            } else { gamepad1.setLedColor(0, 1, 0, -1);}

//            Auto drive to the determine Zones
        if (gamepad1.dpad_up && !trig.ableToShoot(RobotX, RobotY)) {
            Pose2d EnterShootingZone = new Pose2d(RobotStatic.TRIANGLE_X[1],
                    RobotStatic.TRIANGLE_Y[1], Heading);

            Action driveToZone = drive.actionBuilder(getPose)
                    .lineToX(EnterShootingZone.position.x)
                    .lineToY(EnterShootingZone.position.y)
                    .build();
            Actions.runBlocking(new ParallelAction(driveToZone));
        }
        if (gamepad1.dpad_down && !trig.ableToShoot(RobotX, RobotY)) {
            Pose2d EnterShootingZone = new Pose2d(RobotStatic.TRIANGLE_XS[1], RobotStatic.TRIANGLE_YS[1], Heading);

            Action driveToZone = drive.actionBuilder(getPose)
                    .lineToX(EnterShootingZone.position.x)
                    .lineToY(EnterShootingZone.position.y)
                    .build();
            Actions.runBlocking(new ParallelAction(driveToZone));
        }
        if (gamepad1.dpad_left) {
            Pose2d EnterLoadingZone = new Pose2d(RobotStatic.BlueLoadingZone[0], RobotStatic.BlueLoadingZone[1], Heading);

            Action driveToZone = drive.actionBuilder(getPose)
                    .lineToX(EnterLoadingZone.position.x)
                    .lineToY(EnterLoadingZone.position.y)
                    .build();
            Actions.runBlocking(new ParallelAction(driveToZone));
        }
        if (gamepad1.dpad_right) {
            Pose2d EnterLoadingZone = new Pose2d(RobotStatic.RedLoadingZone[0], RobotStatic.RedLoadingZone[1], Heading);

            Action driveToZone = drive.actionBuilder(getPose)
                    .lineToX(EnterLoadingZone.position.x)
                    .lineToY(EnterLoadingZone.position.y)
                    .build();
            Actions.runBlocking(new ParallelAction(driveToZone));
        }

        telemetry.addLine("========== ROBOT STATE ==========");
        telemetry.addData("State", currentState);
        telemetry.addData("Pose", getPose);
        telemetry.addData("X", RobotX);
        telemetry.addData("Y", RobotY);
        telemetry.addData("Heading", Heading);
        telemetry.addData("Distance", distanceTarget);
        telemetry.addLine("========== COLOR SENSORS ==========");
        telemetry.addData("backTop", backTopColor);
        telemetry.addData("backDownColor", backDownColor);
        telemetry.addData("frontSideColor", frontSideColor);
        telemetry.addData("Purple", PurpleColor);
        telemetry.addData("Green", GreenColor);
        telemetry.addData("Unknown", UnknownColor);
        telemetry.addLine("========== HELPER ==========");
        telemetry.update();
    }

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
                   StooperTime.reset();
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
           StooperTime.reset();
           Shooter.setVelocity(0);
           stooperGate.setPosition(RobotStatic.CLOSE_GATE);
       }
    }
}
