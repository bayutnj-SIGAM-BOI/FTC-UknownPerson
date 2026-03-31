package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class turretMain extends OpMode {
    turret turret = new turret();

    @Override
    public void init() {
        turret.initalize(hardwareMap);
    }

    @Override
    public void loop() {
        turret.setAngleAdjuster();
        turret.spinningTurret();
    }
}
