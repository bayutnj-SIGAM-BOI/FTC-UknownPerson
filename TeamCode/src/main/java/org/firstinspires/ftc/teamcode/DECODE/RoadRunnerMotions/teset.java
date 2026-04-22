package org.firstinspires.ftc.teamcode.DECODE.RoadRunnerMotions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous
public class teset extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, beginPose);

        Action test = drive.actionBuilder(beginPose)
                .lineToX(4)
                .build();

        waitForStart();
        Actions.runBlocking(test);
    }
}
