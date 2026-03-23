package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "ForwardTest")
public class Forwardtest extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {


    leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive()) {
            leftDrive.setPower(0.6);
            rightDrive.setPower(0.6);
        }
    }

}
