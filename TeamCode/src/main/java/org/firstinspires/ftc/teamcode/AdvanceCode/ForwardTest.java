package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ForwardTest extends LinearOpMode {
    Config r = new Config();

    @Override
    public void runOpMode() throws InterruptedException {
//        r.initialize(hardwareMap, true, false, false, false);

        waitForStart();

        while (opModeIsActive()) {
            idle();
        }
    }
}
