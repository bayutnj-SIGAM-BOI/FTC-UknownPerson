package org.firstinspires.ftc.teamcode.Decode.Triangle.RoadRunnerMotions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Decode.Triangle.RoadRunnerMotions.RoadRunnerActions.Act;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.Decode.Triangle.RobotStatic;
import org.firstinspires.ftc.teamcode.Decode.Triangle.Turret.TurretWithPoseEstimate;

@Autonomous
public class BlueAutonomous extends LinearOpMode {
    TurretWithPoseEstimate tp;
    Act act;
    RobotStatic rc = new RobotStatic();

    private final Pose2d ShootingPose = new Pose2d(-28.2, -28.2, Math.toRadians(30));
    private final Pose2d firstPose = new Pose2d(-28.0, -28.2, Math.toRadians(90));
    private final Pose2d secondPose = new Pose2d(-28.2, -28.2, Math.toRadians(30));
    private final Pose2d thirdPose = new Pose2d(-28.0, 28.0, Math.toRadians(25));


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-55.5, -55.3, Math.toRadians(45));
        TankDrive drive = new TankDrive(hardwareMap, beginPose);
//        act = new Act(hardwareMap, telemetry);

        tp = new TurretWithPoseEstimate(hardwareMap);
        double Distance = Math.hypot(ShootingPose.position.x - RobotStatic.blueAimingTarget.position.x, ShootingPose.position.y - RobotStatic.blueAimingTarget.position.y);

        Action turretTracking = telemetryPacket -> {
            drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();
            tp.aimingTurret(RobotStatic.blueAimingTarget, pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            return true;
        };

        Action ShooterSeq = drive.actionBuilder(ShootingPose)
                .stopAndAdd(act.shooterWheel(Distance))
                .build();

        Action servoStooper = drive.actionBuilder(ShootingPose)
                .stopAndAdd(act.setStooper(RobotStatic.OPEN_GATE))
                .afterTime(1.5, act.setStooper(RobotStatic.CLOSE_GATE))
                .build();


        Action FirstShooting = drive.actionBuilder(beginPose)
                .lineToX(-28.2)
                .waitSeconds(1.6)
                .build();

        Action FirstBall = drive.actionBuilder(firstPose)
                .turn(Math.toRadians(-95))
                .lineToX(-10.3)
                .lineToX(-28.2)
                .waitSeconds(1.6)
                .build();
        Action SecondBall = drive.actionBuilder(secondPose)
                .turn(Math.toRadians(25))
                .lineToX(12.9)
                .lineToX(-27.7)
                .waitSeconds(1.6)
                .build();
        Action ThirdBall = drive.actionBuilder(thirdPose)
                .build();


        Action FirstShoot = new ParallelAction(
                act.Intake(),
                act.autoAdjust(Distance),
                FirstShooting,
                ShooterSeq
        );
        Action FirstRound = new ParallelAction(
                FirstBall,
                act.autoAdjust(Distance),
                servoStooper
        );

        Action SecondRound = new ParallelAction(
                SecondBall,
                act.autoAdjust(Distance),
                servoStooper
        );

        Action ThirdRound = new ParallelAction(
                ThirdBall,
                act.autoAdjust(Distance),
                servoStooper
        );


        waitForStart();
        Actions.runBlocking(new ParallelAction(
                turretTracking,
                new SequentialAction(FirstShoot, FirstRound, SecondRound, ThirdRound,
                        act.StopShooter(), act.IntakeStop(), act.StopShooter()
                )));

        while (opModeIsActive()) {
            idle();
        }
    }
}
