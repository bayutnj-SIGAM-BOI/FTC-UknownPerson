package org.firstinspires.ftc.teamcode.DECODE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
@Config
public class testIntakeSh extends OpMode {
    private DcMotor Intake;
    private DcMotorEx flyWheel;
    public static double flyWheelspeed = 1700;
    @Override
    public void init() {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flywheelPID =  new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPID);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        Intake.setPower(1);
        flyWheel.setPower(flyWheelspeed);
    }
}
