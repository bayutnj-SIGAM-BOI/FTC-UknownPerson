package org.firstinspires.ftc.teamcode.opMode.AutonomousEncoder.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;

@Autonomous(name = "RedZone", group = "Go2steam")
public class RedZone extends LinearOpMode{

    private DcMotor leftDrive, rightDrive = null;
    private DcMotorEx flyWheel = null;
    private Servo Launcher;
    private final ElapsedTime runtime = new ElapsedTime();

//    Drive Train motors setting
    static final double WHEEL_DIAMETER_INCHES = 3.54331; // 90mm In Inches 3,5 or more
    static final double COUNTS_PER_MOTOR_REV = 288.0; // CoreHex counts per Rev//
    static final double DRIVE_GEAR_REDUCTION = 1.0 / 1.0; // use no gear ration added//

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI); // Formula to convert ticks to inches FROM Google

//    FLyWheels setting
public static double kP = 200;
public static double kF = 12.6;
    double TargetRPM = 1800;
    PIDFCoefficients lastPIDF;



    @Override
    public void runOpMode(){
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        Launcher = hardwareMap.get(Servo.class, "Launcher");


//        set Direction to Reverse
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        Encoder Setup
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lastPIDF = new PIDFCoefficients(kP, 0 ,0 , kF);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastPIDF);


//        Code Here
        waitForStart();
        sleep(1000);
        Shoot3x(); //Shoot the 3artifacts at once

        telemetry.addData("Path", "Complete");
        telemetry.update();

        while(opModeIsActive()) {
            idle();
        }
    }

//    setup Movement with Encoder
    private void driveEncoders(double leftTarget, double rightTarget, double speed, long TimeoutSecs) {
        int newleftTarget;
        int newrightTarget;

        if (opModeIsActive()) {
            newleftTarget = leftDrive.getCurrentPosition() + (int)(leftTarget * COUNTS_PER_INCH);
            newrightTarget = rightDrive.getCurrentPosition() + (int)(rightTarget * COUNTS_PER_INCH);

//            Start pos
            leftDrive.setTargetPosition(newleftTarget);
            rightDrive.setTargetPosition(newrightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.milliseconds() <TimeoutSecs && leftDrive.isBusy() && rightDrive.isBusy()) {
               telemetry.addData("Running to", " %7d :%7d", newleftTarget, newrightTarget);
               telemetry.addData("Current Position", " %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
               telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private void Shooters() {
        PIDFCoefficients current = new PIDFCoefficients(kP, 0 ,0 , kF);
        if (!current.equals(lastPIDF)) {
            flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, current);
            lastPIDF = current;
        }
        if (opModeIsActive()) {

            sleep(450);
         flyWheel.setVelocity(TargetRPM);
         sleep(900);
         Launcher.setPosition(0.5);

         sleep(300);
         Launcher.setPosition(0.875);
         flyWheel.setVelocity(0);


         telemetry.addData("Flywheel Velocity : ", flyWheel.getVelocity());
         telemetry.update();
        }
    }

    private void Shoot3x() {
        for (int a = 0; a < 4;  a++) {
            Shooters();
            sleep(300);
        }
    }
}