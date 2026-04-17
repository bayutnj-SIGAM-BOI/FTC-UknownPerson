package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class roadrunnerTest extends LinearOpMode {
    TankDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(new Vector2d(0, 0), 0);

        TankDrive drive = new TankDrive(hardwareMap, beginPose);

        Action testAction = drive.actionBuilder(beginPose)
                .lineToX(24)
                .turnTo(Math.toRadians(90))
                .lineToX(-28.2)
                .build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(
                testAction
        ));

        while (opModeIsActive()) {

        }
    }
}
