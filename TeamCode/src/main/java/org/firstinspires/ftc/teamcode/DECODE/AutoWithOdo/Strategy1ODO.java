//package org.firstinspires.ftc.teamcode.DECODE.AutoWithOdo;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.DECODE.PIDControl;
//import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
//import org.firstinspires.ftc.teamcode.DECODE.Turret.TurretWithPoseEstimate;
//
//
//@Config
//@Autonomous(name = "EX PINPOINT AUTO #1", group = "EXHIBITION")
//public class Strategy1ODO extends OpMode {
//    private GoBildaPinpointDriver odo;
//    private TurretWithPoseEstimate turret;
//    private RobotStatic rc = new RobotStatic();
//    private Servo hoodAngle;
//    private DcMotor Intake;
//    private DcMotorEx flyWheel;
//    public static double pidXP = 0.15, pidXI, pidXD = 0.008;
//    public static double pidYP = 0.15, pidYI, pidYD = 0.008;
//    public static double pidHP = 0.8, pidHI = 0, pidHD = 0.002;
//
//    private PIDControl pidX = new PIDControl(pidXP, pidXI, pidXD);
//    private PIDControl pidY = new PIDControl(pidYP, pidYI, pidYD);
//    private PIDControl pidH = new PIDControl(pidHP, pidHI, pidHD);
//
//    private DcMotor leftFront, rearLeft, rightFront, rearRight;
//
//    private enum autoState {
//        WAITING_FOR_START, MOVE_1, MOVE_2, DONE
//    }
//
//    autoState currentState = autoState.MOVE_1;
//
//    private final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
//    private final Pose2D move_1 = new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0);
//    private final Pose2D move_2 = new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0);
////    private final Pose2D move_3 = new Pose2D(DistanceUnit.INCH, -48, 30, AngleUnit.DEGREES, 0);
////    private final Pose2D move_4 = new Pose2D(DistanceUnit.INCH, 0, -42, AngleUnit.DEGREES, 0);
////    private final Pose2D move_5 = new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 0);
////    private final Pose2D move_6 = new Pose2D(DistanceUnit.INCH, -48, 42, AngleUnit.DEGREES, 0);
//
//    @Override
//    public void init() {
//        turret = new TurretWithPoseEstimate(hardwareMap);
//
//        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
//        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
//
//        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        Intake = hardwareMap.get(DcMotor.class, "Intake");
//        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
//        PIDFCoefficients flywheelPID = new PIDFCoefficients(200, 0, 0, 13.1000);
//        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPID);
//        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
//
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(-7.5, 8.5, DistanceUnit.CM);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();
//        odo.setPosition(START_POSE);
//    }
//
//    @Override
//    public void loop() {
//        odo.update();
//        Pose2D pos = odo.getPosition();
//        double dx = pos.getX(DistanceUnit.INCH); // Forward
//        double dy = pos.getY(DistanceUnit.INCH); // Strafe
//        double rh = pos.getHeading(AngleUnit.RADIANS);
//        double dist = Math.hypot(dx - RobotStatic.blueAimingTarget.position.x, dy - RobotStatic.blueAimingTarget.position.y);
//
//        turret.aimingTurret(RobotStatic.blueAimingTarget, dx, dy, rh, );
//        hoodAngle.setPosition(rc.hoodAngle(dist));
//
//        switch (currentState) {
//            case WAITING_FOR_START:
//                currentState = autoState.MOVE_1;
//                break;
//            case MOVE_1:
////                Intake.setPower(RobotStatic.INTAKE_SPEED);
////                flyWheel.setVelocity(rc.flywheelSpeed(dist));
//                if (driveTP(move_1, dx, dy, rh)) {
//                    currentState = autoState.DONE  ;
//                }
//                break;
//
////            case MOVE_2:
//////                Intake.setPower(RobotStatic.INTAKE_SPEED);
////                flyWheel.setVelocity(0);
////                if (driveTP(move_2, dx, dy, rh)){
////                    currentState = autoState.DONE; }
////                break;
//
//            case DONE:
//                leftFront.setPower(0);
//                rightFront.setPower(0);
//                rearLeft.setPower(0);
//                rearRight.setPower(0);
//                telemetry.addLine("AUTONOMOUS DONE");
//                telemetry.update();
//                break;
//        }
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("AUTO STATE", currentState);
//        packet.put("SHOOTER VELOCITY", flyWheel.getVelocity());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        telemetry.addData("State", currentState);
//        telemetry.update();
//    }
//
//    private boolean driveTP(Pose2D targetPos, double dx, double dy, double rh) {
//        double targetX = targetPos.getX(DistanceUnit.INCH);
//        double targetY = targetPos.getY(DistanceUnit.INCH);
//        double targetDeg = targetPos.getHeading(AngleUnit.DEGREES);
//        double targetRad = Math.toRadians(targetDeg);
//
//        double eX = targetX - dx;
//        double eY = targetY - dy;
//        double eH = pidH.angleWrapRadians(targetRad - rh);
//
//        boolean xFin = Math.abs(eX) < 2.3;
//        boolean yFin = Math.abs(eY) < 2.3;
//        boolean hFin = Math.abs(eH) < 0.035;
//        if (xFin && yFin && hFin) {
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            rearLeft.setPower(0);
//            rearRight.setPower(0);
//            pidX.reset();
//            pidY.reset();
//            pidH.reset();
//            return true;
//        }
//
//        double calX = pidX.calculateInches(targetX, dx);
//        double calY = pidY.calculateInches(targetY, dy);
//        double calH = pidH.calculateRadians(targetRad, rh);
//
//        if (xFin) calX = 0;
//        if (yFin) calY = 0;
//        if (hFin) calH = 0;
//
//        double cos = Math.cos(rh);
//        double sin = Math.sin(rh);
//        double rotX = calX * cos - calY * sin;
//        double rotY = calX * sin + calY * cos;
//
//        double dist = Math.hypot(eX, eY);
//        double sspeed = Math.min(dist / 24.0, 1.0);
//
//        rotX *= sspeed;
//        rotY *= sspeed;
//
//        double clamp = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(calH), 1.0);
//        leftFront.setPower((rotY - rotX + calH) / clamp);
//        rearLeft.setPower((rotY + rotX + calH) / clamp);
//        rightFront.setPower((rotY + rotX - calH) / clamp);
//        rearRight.setPower((rotY - rotX - calH) / clamp);
//
//        telemetry.addData("eX", eX);
//        telemetry.addData("eY", eY);
//        telemetry.addData("CURRENT X", dx);
//        telemetry.addData("CURRENT Y", dy);
//        telemetry.addData("CURRENT H", rh);
//        telemetry.addData("eH deg", Math.toDegrees(eH));
//        telemetry.addData("LF POWER", leftFront.getPower());
//        telemetry.addData("LB POWER", rearLeft.getPower());
//        telemetry.addData("RF POWER", rightFront.getPower());
//        telemetry.addData("RB POWER", rearRight.getPower());
//        telemetry.update();
//
//        TelemetryPacket packet = new TelemetryPacket();
//
//        packet.put("targetY", targetY);
//        packet.put("currentY", dy);
//        packet.put("errorY", eY);
//        packet.put("calY", calY);
//
//        packet.put("targetX", targetX);
//        packet.put("currentX", dx);
//        packet.put("errorX", eX);
//
//        packet.put("heading", rh);
//        packet.put("errorH", eH);
//
//        packet.put("xFin", xFin);
//        packet.put("yFin", yFin);
//        packet.put("hFin", hFin);
//        packet.put("eH deg", Math.toDegrees(eH));
//
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        return false;
//    }
//}
