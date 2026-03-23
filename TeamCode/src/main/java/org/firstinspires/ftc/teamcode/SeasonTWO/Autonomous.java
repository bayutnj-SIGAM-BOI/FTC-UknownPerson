package org.firstinspires.ftc.teamcode.SeasonTWO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    Config autoConf = new Config();
    ElapsedTime armTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        autoConf.initialize(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            autoConf.resetAndStraightTo(90, 0);

            autoConf.turnTo(45);

            while (opModeIsActive() && armTimer.seconds() < 2.0) {
                autoConf.armTo(90);
            }
            autoConf.turnTo(-45);

            autoConf.resetAndStraightTo(-90, 0);
            autoConf.resetAndStraightTo(-90, 20);

            autoConf.moveToDegIn(45, 24, 12);
            autoConf.moveToInDeg(24, 45, -12);
        }
    }
}
