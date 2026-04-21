package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class servoTest extends OpMode {
    Servo servo;
    double[] pos = {-0.1, -0, 2 - 0.3, 0.4, -0.5, -0.6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    int index;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            index = (index + 1) % pos.length;
        }

        if (gamepad1.left_bumper) {
            servo.setPosition(pos[index]);
        }

        telemetry.addData("index", index);
        telemetry.update();
    }
}
