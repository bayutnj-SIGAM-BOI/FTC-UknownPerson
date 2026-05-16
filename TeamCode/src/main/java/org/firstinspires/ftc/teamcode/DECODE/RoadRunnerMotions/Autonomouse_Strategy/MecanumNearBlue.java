//package org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions.Autonomouse_Strategy;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SplineHeadingPath;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.robot.Robot;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions.RoadRunnerActions.Act;
//import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
//import org.firstinspires.ftc.teamcode.DECODE.Turret.TurretWithPoseEstimate;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//public class MecanumNearBlue extends LinearOpMode {
//    MecanumDrive drive;
//    TurretWithPoseEstimate turret;
//    RobotStatic rc;
//    Act action;
//
////    Poses Needed in the match
//    Pose2d beginPose = new Pose2d(-55.3, -55.2, Math.toRadians(-90));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new MecanumDrive(hardwareMap, beginPose);
//        turret = new TurretWithPoseEstimate(hardwareMap);
//        action = new Act(hardwareMap, telemetry);
//
//        waitForStart();
//        drive.localizer.update();
//        Pose2d getPose = drive.localizer.getPose();
//        double x = getPose.position.x;
//        double y = getPose.position.y;
//        double h = getPose.heading.toDouble();
//        double distanceTarget = Math.hypot(x - RobotStatic.blueAimingTarget.position.x, y - RobotStatic.blueAimingTarget.position.y);
//
////        Subsystem
//        Action turretTracking = telemetryPacket -> {
//            turret.aimingTurret(RobotStatic.blueAimingTarget, x, y, h);
//            return true;
//        };
//
//        Action Shooter = drive.actionBuilder(getPose)
//                .stopAndAdd(action.shooterWheel(distanceTarget))
//                .build();
//        Action servoGate = drive.actionBuilder(getPose)
//                .stopAndAdd(action.setStooper(RobotStatic.OPEN_GATE))
//                .afterTime(1, action.setStooper(RobotStatic.CLOSE_GATE))
//                .build();
//        Action angleAdjusting = drive.actionBuilder(getPose)
//                .stopAndAdd(action.autoAdjust(distanceTarget))
//                .build();
//
////        Drive base trajectory
//        Action tab1 = drive.actionBuilder(getPose)
//                .strafeToLinearHeading(new Vector2d(-16.0, -16.4), Math.toRadians(-45))
//                .waitSeconds(1.5)
//                .build();
//        Action tab2 = drive.actionBuilder(getPose)
//                .strafeToLinearHeading(new Vector2d(13.4, -47.8), Math.toRadians(30))
//                .waitSeconds(0.3)
//                .strafeToConstantHeading(new Vector2d(-16.0, -16.4))
//                .waitSeconds(1.5)
//                .build();
//        Action tab3 = drive.actionBuilder(getPose)
//                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
//                .splineToLinearHeading(new Pose2d(new Vector2d(-4.1, -65.0), 10), 10)
//                .waitSeconds(3)
//                .build();
//        Action tab4 = drive.actionBuilder(getPose)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
//                .waitSeconds(1.5)
//                .build();
//        Action tab5 = drive.actionBuilder(getPose)
//                .setTangent(5)
//                .strafeToLinearHeading(new Vector2d(-12.0, -34.5),Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(-12.0, -55.0))
//                .build();
//        Action tab6 = drive.actionBuilder(getPose)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-23.3, -23.4), 8)
//                .waitSeconds(0.4)
//                .build();
//        Action tab7 = drive.actionBuilder(getPose)
//                .strafeToConstantHeading(new Vector2d(34.4, -33.1))
//                .strafeToConstantHeading(new Vector2d(34.7, -52.3))
//                .build();
//        Action tab8 = drive.actionBuilder(getPose)
//                .setReversed(true)
//                .strafeToConstantHeading(new Vector2d(-24.5, -9.2))
//                .build();
//
//        Action Ptab1 = new ParallelAction( tab1, servoGate );
//        Action Ptab2 = new ParallelAction( tab2, servoGate );
//        Action Ptab4 = new ParallelAction( tab4, servoGate );
//        Action Ptab6 = new ParallelAction( tab6, servoGate );
//        Action Ptab8 = new ParallelAction( tab8, servoGate );
//        Actions.runBlocking(new ParallelAction(turretTracking, angleAdjusting, Shooter, new SequentialAction(Ptab1, Ptab2, tab3, Ptab4, tab5, Ptab6, tab7, Ptab8)));
//
//        while(opModeIsActive()) {
//            idle();
//        }
//    }
//}
