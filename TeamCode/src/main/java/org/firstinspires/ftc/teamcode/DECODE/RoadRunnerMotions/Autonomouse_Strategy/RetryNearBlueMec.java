package org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions.Autonomouse_Strategy;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions.RoadRunnerActions.Act;
import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
import org.firstinspires.ftc.teamcode.DECODE.Turret.TurretWithPoseEstimate;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RetryNearBlueMec extends LinearOpMode {
    MecanumDrive drive;
    TurretWithPoseEstimate turret;
    Act action;

    //    Poses Needed in the match
    Pose2d beginPose = new Pose2d(-55.3, -55.2, Math.toRadians(-90));
    Pose2d endTab1 = new Pose2d(-41.4, -40.7, Math.toRadians(-20));
    Pose2d endTab2 = new Pose2d(-41.4, -40.7, Math.toRadians(-20));
    Pose2d endTab3 = new Pose2d(-23.1, -23.1, Math.toRadians(5));
    Pose2d endTab4 = new Pose2d(-16.0, -16.4, Math.toRadians(11));
    Pose2d endTab5 = new Pose2d(-16.0, -16.4, Math.toRadians(11));


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, beginPose);
        turret = new TurretWithPoseEstimate(hardwareMap);
        action = new Act(hardwareMap, telemetry);

//        Subsystem
        Action turretTracking = telemetryPacket -> {
            drive.localizer.update();
            Pose2d getpose = drive.localizer.getPose();
            double x = getpose.position.x;
            double y = getpose.position.y;
            double h = getpose.heading.toDouble();
            turret.aimingTurret(RobotStatic.blueAimingTarget, x, y, h);
            return true;
        };

       Action Shooter = telemetryPacket -> {
           drive.localizer.update();
           Pose2d getpose = drive.localizer.getPose();
           double x = getpose.position.x;
           double y = getpose.position.y;
           double h = getpose.heading.toDouble();
           double dist = Math.hypot(x - RobotStatic.blueAimingTarget.position.x, y - RobotStatic.blueAimingTarget.position.y);
           action.shooterWheel(dist);
           return true;
       };

        Action servoGate = drive.actionBuilder(beginPose)
                .stopAndAdd(action.setStooper(RobotStatic.OPEN_GATE))
                .afterTime(1, action.setStooper(RobotStatic.CLOSE_GATE))
                .build();
        Action angleAdjusting = telemetryPacket -> {
            drive.localizer.update();
            Pose2d getpose = drive.localizer.getPose();
            double x = getpose.position.x;
            double y = getpose.position.y;
            double h = getpose.heading.toDouble();
            double dist = Math.hypot(x - RobotStatic.blueAimingTarget.position.x, y - RobotStatic.blueAimingTarget.position.y);
            action.autoAdjust(dist);
            return true;
        };

//        Drive base trajectory
        Action tab1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-41.4, -40.7), Math.toRadians(-20))
                .waitSeconds(0.4)
                .build();
        Action tab2 = drive.actionBuilder(endTab1)
                .strafeToConstantHeading(new Vector2d(-6.9, -46.4))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-41.4, -40.7))
                .waitSeconds(0.4)
                .build();
        Action tab3 = drive.actionBuilder(endTab2)
                .strafeToLinearHeading(new Vector2d(17.1, -46.0), Math.toRadians(5))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-23.1, -23.1))
                .waitSeconds(0.4)
                .build();
        Action tab4 = drive.actionBuilder(endTab3)
                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                .splineToLinearHeading(new Pose2d(new Vector2d(-4.1, -65.0), 10), 10)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                .waitSeconds(0.4)
                .build();
        Action tab5 = drive.actionBuilder(endTab4)
                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                .splineToLinearHeading(new Pose2d(new Vector2d(-4.1, -65.0), 10), 10)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                .waitSeconds(0.4)
                .build();
        Action tab6 = drive.actionBuilder(endTab5)
                .strafeToLinearHeading(new Vector2d(37.9, -46.5), Math.toRadians(-15))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-24.5, -9.2))
                .build();

        waitForStart();
        Actions.runBlocking(new ParallelAction(turretTracking, angleAdjusting, Shooter, new SequentialAction(tab1, servoGate, tab2, servoGate, tab3, servoGate, tab4, servoGate, tab5, servoGate, tab6, servoGate)));

        while(opModeIsActive()) {
            idle();
        }
    }
}