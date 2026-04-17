package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTest extends LinearOpMode {
    private DcMotor Encodertest;

    @Override
    public void runOpMode() throws InterruptedException {
        Encodertest = hardwareMap.get(DcMotor.class, "EncoderTest");
        Encodertest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double encoder = Encodertest.getCurrentPosition();
            telemetry.addData("ENC", encoder);
            telemetry.update();
        }
    }
}
