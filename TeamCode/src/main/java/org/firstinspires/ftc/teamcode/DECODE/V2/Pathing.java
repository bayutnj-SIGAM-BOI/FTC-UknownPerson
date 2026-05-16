package org.firstinspires.ftc.teamcode.DECODE.V2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Pathing extends LinearOpMode {
    MecanumDrive drive;

    private final Pose2d StartPose = (new Pose2d(-55.3, -55.2, -90));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, StartPose);

        Action motion = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(-41.4, -40.7), Math.toRadians(-20))
                .waitSeconds(0.4)

                .strafeToConstantHeading(new Vector2d(-11.5, -47.2))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-41.4, -40.7))
                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(17.1, -46.0), Math.toRadians(5))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-23.1, -23.1))
                .waitSeconds(0.4)

                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                .splineToLinearHeading(new Pose2d(new Vector2d(5.8, -59.6), 10), 10)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                .waitSeconds(0.4)

                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                .splineToLinearHeading(new Pose2d(new Vector2d(5.8, -59.6), 10), 10)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(37.5, -47.4), Math.toRadians(-15))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-24.5, -9.2))
                .build();

        waitForStart();
        PosesStorage.currentPose = drive.localizer.getPose();
        Actions.runBlocking(new SequentialAction(motion));
    }
}
