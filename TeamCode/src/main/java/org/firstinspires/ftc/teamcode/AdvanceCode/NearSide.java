//package org.firstinspires.ftc.teamcode.AdvanceCode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous
//public class NearSide extends LinearOpMode {
//    Config r = new Config();
//    turret turret = new turret();
//    ElapsedTime shootingTimer = new ElapsedTime();
//    ElapsedTime intakeTimer = new ElapsedTime();
//
//    enum State {
//        RESET_ENCODER,
//        BACKWARD,
//        INTAKE_SHOOTING,
//        TURN,
//        BACKWARD_FIRST_BALL,
//        TURN_FACING_FIRST_BALL,
//        FORWARD_INTAKE_1,
//        BACKWARD_INTAKE_1,
//        TURN_TO_MIN30,
//        FORWARD_SHOOTING_ZONE,
//        CASE_DONE
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        turret.initalize(hardwareMap, telemetry);
/// /        r.initialize(hardwareMap, true, false, false, true);
//        turret.spinTimer.reset();
//
//        waitForStart();
//        State currentState = State.RESET_ENCODER;
//        turret.updateWebcam();
//        while (opModeIsActive()) {
//            switch (currentState) {
//                case RESET_ENCODER:
//                    r.resetAllEncoders();
//                    shootingTimer.reset();
//                    intakeTimer.reset();
//                    telemetry.addData("State", "Resetting all Motor Encoders");
//                    telemetry.update();
//                    currentState = State.BACKWARD;
//                    break;
//
//                case BACKWARD:
//                    if (r.straightTo(-72, 0)) {
//                        telemetry.addData("State", "Backward -72");
//                        telemetry.update();
//                        currentState = State.INTAKE_SHOOTING;
//                    }
//                    break;
//
//                case INTAKE_SHOOTING:
//                    r.Intake(1800);
//                    turret.setVelocityAuto();
//                    turret.setStooperOpen();
//                    if (intakeTimer.seconds() > 2) {
//                        turret.turretWheel.setVelocity(0);
//                        turret.setStooperClose();
//                        telemetry.addData("Shooting", "Early Balls");
//                        currentState = State.TURN;
//                    }
//                    break;
//
//                case TURN:
//                    if (r.turnTo(-30)) {
//                        telemetry.addData("State", "Turn 30°");
//                        telemetry.update();
//                        currentState = State.BACKWARD_FIRST_BALL;
//                    }
//                    break;
//
//                case BACKWARD_FIRST_BALL:
//                    if (r.straightTo(-36, 0)) {
//                        telemetry.addData("State", "Backward -36");
//                        telemetry.update();
//                        currentState = State.TURN_FACING_FIRST_BALL;
//                    }
//                    break;
//
//                case TURN_FACING_FIRST_BALL:
//                    if (r.turnTo(90)) {
//                        telemetry.addData("State", "Turn 90°");
//                        telemetry.update();
//                        currentState = State.FORWARD_INTAKE_1;
//                    }
//                    break;
//
//                case FORWARD_INTAKE_1:
//                    if (r.straightTo(30, 0)) {
//                        telemetry.addData("State", "Straight 30");
//                        telemetry.update();
//                        currentState = State.BACKWARD_INTAKE_1;
//                    }
//                    break;
//
//                case BACKWARD_INTAKE_1:
//                    if (r.straightTo(-30, 0)) {
//                        telemetry.addData("State", "Straight -30");
//                        telemetry.update();
//                        currentState = State.TURN_TO_MIN30;
//                    }
//                    break;
//
//                case TURN_TO_MIN30:
//                    if (r.turnTo(-30)) {
//                        shootingTimer.reset();
//                        telemetry.addData("State", "Turn -30°");
//                        telemetry.update();
//                        currentState = State.FORWARD_SHOOTING_ZONE;
//                    }
//                    break;
//
//                case FORWARD_SHOOTING_ZONE:
//                    if (r.straightTo(36, 0)) {
//                        telemetry.addData("State", "Backward 36");
//                        telemetry.update();
//                        turret.setVelocityAuto();
//
//                        if (shootingTimer.seconds() > 2) {
//                            turret.turretWheel.setVelocity(0);
//                            r.Intake(0);
//                            currentState = State.CASE_DONE;
//                        }
//                    }
//                    break;
//
//                case CASE_DONE:
//                    telemetry.addData("ALL DONE", "MOTIONS HAS BEEN COMPLETED");
//                    break;
//            }
//        }
//    }
//}
