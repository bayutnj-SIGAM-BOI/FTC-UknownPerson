package org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner.Component.ShooterAction;

@Autonomous(name = "AutoRoadRunner", group = "Go2steam")
public class roadrunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(new Vector2d(58.8, -9.2), Math.toRadians(0));
//        Pose2d beginPose = new Pose2d(new Vector2d(0,0), Math.toRadians(0));
        TankDrive drive = new TankDrive(hardwareMap, beginPose);
        ShooterAction shooter = new ShooterAction(hardwareMap);

        telemetry.addLine("========= Autonomous Mode =========");
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.1f°)", beginPose.position.x, beginPose.position.y, Math.toDegrees(beginPose.heading.toDouble()));
        telemetry.update();

        Action path = drive.actionBuilder(beginPose)
                .lineToX(-0.2)
                .turn(Math.toRadians(50))
                .build();

        Action auto = new SequentialAction(
                new ParallelAction(
                        path,
                        shooter.SpinUp()
                ),
                shooter.multiShoot(3, 1.000),
                shooter.stopAction()
        );

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addLine("Starting path");

        Actions.runBlocking(new SequentialAction(auto));
    }
}