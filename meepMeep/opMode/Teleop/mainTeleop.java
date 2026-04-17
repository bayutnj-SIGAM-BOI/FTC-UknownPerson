package org.firstinspires.ftc.teamcode.opMode.Teleop;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Teleop", group = "Go2Steam")
public class mainTeleop extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx flyWheel = null;
    private Servo Launcher;
    boolean autoAlign = false;
    double targetRPM = 1800;
    boolean lastY = false;
    boolean ShooterOn = false;
    boolean LauncherLock = true;
    PIDFCoefficients lastPIDF;
    public static double kP = 200;
    public static double kF = 12.74;

    // Auto align
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // PD Controller
    public static double kPAuto = 0.14;
    public static double kD = 0.21;
    double error = 0;
    double lastError = 0;
    double[] goalX = {0, 3, -3, 6, -6, 10, -10}; //Offset
    int index = 0;
    //    double goalX = 0;
    double angleTolerance = 0.4;
    double curTime = 0;
    double lastTime = 0;

    // Driving Setup
    double rotate, forward;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

//        ================ Hardware initialize ================
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");

        Launcher = hardwareMap.get(Servo.class, "Launcher");

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launcher.setPosition(0.875);
        lastPIDF = new PIDFCoefficients(kP, 0, 0, kF);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastPIDF);
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }


    @Override
    public void loop() {
        double leftPower, rightPower;

        forward = -gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x;
        double speedMultiply = gamepad1.right_bumper ? 0.6 : 1;

        //get Apriltag Info
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpesificId(21);
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpesificId(22);

        if (gamepad1.xWasPressed()) {
            index = (index + 1) % goalX.length;
        }

        boolean isAutoAligning = false;
        AprilTagDetection targetTag = null;

        if (gamepad1.left_bumper && id20 != null) {
            targetTag = id20;
            isAutoAligning = true;
            autoAlign = true;
        } else if (gamepad1.left_trigger > 0.5 && id21 != null) {
            targetTag = id21;
            isAutoAligning = true;
            autoAlign = true;
        } else {
            autoAlign = false;
        }

        if (isAutoAligning) {
            if (targetTag != null) {
                error = goalX[index] - targetTag.ftcPose.bearing;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kPAuto;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.5, 0.5);


                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        leftPower = Range.clip(forward - rotate, -1.0, 1.0);
        rightPower = Range.clip(forward + rotate, -1.0, 1.0);

        leftDrive.setPower(leftPower * speedMultiply);
        rightDrive.setPower(rightPower * speedMultiply);


        // Shooter
        if (!ShooterOn && !LauncherLock) { // So if Shooter is not on and LaunchLock is false, LaunchLock will be true, which makes the Launcher not work;
            LauncherLock = true;
        } else if (gamepad1.a && ShooterOn) { // And if gamepad1.a is pressed && the Shoter is active, you can use the Launcher
            LauncherLock = false;
            Launcher.setPosition(0.5);
            sleep(300);
            Launcher.setPosition(0.875);
        }

        if (gamepad1.y && !lastY) {
            ShooterOn = !ShooterOn;
        }
        lastY = gamepad1.y;


        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM = 1500;
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetRPM += 50
            ;
        }

        if (gamepad1.dpadDownWasPressed()) {
            targetRPM -= 50;
        }


        PIDFCoefficients current = new PIDFCoefficients(kP, 0, 0, kF);
        if (!current.equals(lastPIDF)) {
            flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, current);
            lastPIDF = current;
        }
        targetRPM = Range.clip(targetRPM, 1400, 1800);

        if (ShooterOn) {
            flyWheel.setVelocity(targetRPM);
        } else {
            flyWheel.setVelocity(0);
        }

        telemetry.addData("MODE", autoAlign ? "AUTO ALIGN" : "MANUAL");
        telemetry.addData("Shooter Status", ShooterOn ? "ON" : "OFF");
        telemetry.addData("Launcher Status", LauncherLock ? "LOCKED" : "READY");
        telemetry.addData("Left Power", "%.2f", leftDrive.getPower());
        telemetry.addData("Right Power", "%.2f", rightDrive.getPower());
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Actual RPM", "%.0f", flyWheel.getVelocity());
        telemetry.addData("kF", "%.2f", kF);

        telemetry.addLine("\n===== APRILTAG WEBCAM =====");
        telemetry.addData("Total Tags Detected", aprilTagWebcam.getDetectedTagCount());
        telemetry.addData("ID 20 Status", id20 != null ? "✓ DETECTED" : "✗ Not Found");
        telemetry.addData("ID 24 Status", id21 != null ? "✓ DETECTED" : "✗ Not Found");

        if (id20 != null) {
            telemetry.addLine("\n--- AprilTag ID 20 ---");
            telemetry.addData("  Bearing", "%.2f°", id20.ftcPose.bearing);
            telemetry.addData("  Range", "%.1f cm", id20.ftcPose.range);
            telemetry.addData("  Position X", "%.1f cm", id20.ftcPose.x);
            telemetry.addData("  Position Y", "%.1f cm", id20.ftcPose.y);
            telemetry.addData("  Position Z", "%.1f cm", id20.ftcPose.z);
            telemetry.addData("  Yaw", "%.1f°", id20.ftcPose.yaw);
        }

        if (id21 != null) {
            telemetry.addLine("\n--- AprilTag ID 24 ---");
            telemetry.addData("  Bearing", "%.2f°", id21.ftcPose.bearing);
            telemetry.addData("  Range", "%.1f cm", id21.ftcPose.range);
            telemetry.addData("  Position X", "%.1f cm", id21.ftcPose.x);
            telemetry.addData("  Position Y", "%.1f cm", id21.ftcPose.y);
            telemetry.addData("  Position Z", "%.1f cm", id21.ftcPose.z);
            telemetry.addData("  Yaw", "%.1f°", id21.ftcPose.yaw);
        }

        if (autoAlign) {
            telemetry.addLine("\n===== AUTO ALIGN DATA =====");
            telemetry.addData("Active Target", targetTag != null ? "ID " + targetTag.id : "None");
            telemetry.addData("Goal Offset", "%.2f°", goalX[index]);
            telemetry.addData("Current Error", "%.2f°", error);
            telemetry.addData("Rotate Power", "%.3f", rotate);
            telemetry.addData("Within Tolerance", Math.abs(error) < angleTolerance ? "YES" : "NO");
        }

        telemetry.addLine("\n===== CONTROLS =====");
        telemetry.addData("LB (ID 20)", "Auto Align");
        telemetry.addData("LT (ID 24)", "Auto Align");
        telemetry.addData("X Button", "Cycle Offset");
        telemetry.addData("Current Offset", index + " (" + goalX[index] + "°)");

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}