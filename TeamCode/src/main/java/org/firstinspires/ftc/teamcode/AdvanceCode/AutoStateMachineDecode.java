package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoStateMachineDecode extends LinearOpMode {
    Config autoConf = new Config();
    ElapsedTime ShootingTimer = new ElapsedTime();

    enum State {
        RESET_ENCODERS,
        FACE_GOAL_1,
        SHOOTING_1,
        FACE_STRAIGHT,
        FORWARD_TURN_RIGHT,
        INTAKE,
        FORWARD_2,
        BACKWARD_1,
        BACKWARD_HEADING,
        SHOOTING_2,
        STOP,
    }


    @Override
    public void runOpMode() throws InterruptedException {
        autoConf.initialize(hardwareMap);

        waitForStart();
        telemetry.addData("Initialize", "Success!, Ready TOGOOOO!!!");
        State currentState = State.RESET_ENCODERS;
        while (opModeIsActive()) {
            switch (currentState) {
                case RESET_ENCODERS:
                    autoConf.resetAllEncoders();
                    currentState = State.FACE_GOAL_1;
                    telemetry.addData("Current State : ", State.RESET_ENCODERS);
                    telemetry.update();
                    break;

                case FACE_GOAL_1:
                    if (autoConf.turnTo(30)) {
                        ShootingTimer.reset();
                        telemetry.addData("Current State : ", State.FACE_GOAL_1);
                        telemetry.update();
                        currentState = State.SHOOTING_1;
                    }
                    break;

                case SHOOTING_1:
                    autoConf.Shooter(1800);
                    if (ShootingTimer.seconds() > 1.5) {
                        autoConf.Shooter(0);
                        telemetry.addData("Current State : ", State.SHOOTING_1);
                        telemetry.update();
                        currentState = State.FACE_STRAIGHT;
                    }
                    break;

                case FACE_STRAIGHT:
                    if (autoConf.resetAndTurnTo(-30)) {
                        telemetry.addData("Current State : ", State.FACE_STRAIGHT);
                        telemetry.update();
                        currentState = State.FORWARD_2;
                    }
                    break;

                case FORWARD_TURN_RIGHT:
                    if (autoConf.moveToInDeg(36, 0, 90)) {
                        telemetry.addData("Current State : ", State.FORWARD_TURN_RIGHT);
                        telemetry.update();
                        currentState = State.INTAKE;
                    }
                    break;

                case INTAKE:
                    currentState = State.FORWARD_2;
                    break;

                case FORWARD_2:
                    if (autoConf.resetAndStraightTo(72, 0)) {
                        telemetry.addData("Current State : ", State.FORWARD_2);
                        telemetry.update();
                        currentState = State.BACKWARD_1;
                    }
                    break;

                case BACKWARD_1:
                    if (autoConf.resetAndStraightTo(-72, 0)) {
                        telemetry.addData("Current State : ", State.BACKWARD_1);
                        telemetry.update();
                        currentState = State.BACKWARD_HEADING;
                    }
                    break;

                case BACKWARD_HEADING:
                    if (autoConf.resetAndStraightTo(-36, 50)) {
                        ShootingTimer.reset();
                        telemetry.addData("Current State : ", State.BACKWARD_HEADING);
                        telemetry.update();
                        currentState = State.SHOOTING_2;
                    }
                    break;

                case SHOOTING_2:
                    autoConf.Shooter(1800);
                    if (ShootingTimer.seconds() > 1.5) {
                        autoConf.Shooter(0);
                        telemetry.addData("Current State : ", State.SHOOTING_2);
                        telemetry.update();
                        currentState = State.STOP;
                    }
                    break;

                case STOP:
                    telemetry.addData("Finished", "Motions are already done");
                    telemetry.update();
                    break;
            }
        }
    }
}
