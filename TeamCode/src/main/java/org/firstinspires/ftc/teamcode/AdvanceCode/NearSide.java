package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class NearSide extends LinearOpMode {
    Config r = new Config();
    ElapsedTime shootingTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    enum State {
        RESET_ENCODER,
        BACKWARD,
        INTAKE_SHOOTING,
        TURN,
        BACKWARD_FIRST_BALL,
        TURN_FACING_FIRST_BALL,
        INTAKE,
        FORWARD_INTAKE_1,
        BACKWARD_INTAKE_1,
        CASE_DONE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(hardwareMap, true, false, false, false);

        waitForStart();
        State currentState = State.BACKWARD;
        while (opModeIsActive()) {
            switch (currentState) {
                case RESET_ENCODER:
                    r.resetAllEncoders();
                    currentState = State.BACKWARD;
                    break;

                case BACKWARD:
                    if (r.straightTo(36, 0)) {
                        shootingTimer.reset();
                        currentState = State.CASE_DONE;
                    }
                    break;

//                case INTAKE_SHOOTING:
//                    r.Shooter(1500);
//                    if (shootingTimer.seconds() > 1) {
//                        r.Shooter(0);
//                        currentState = State.TURN;
//                    }
//                    break;
//
//                case TURN:
//                    if (r.turnTo(40)) {
//                        currentState = State.BACKWARD;
//                    }
//                    break;
//
//                case BACKWARD_FIRST_BALL:
//                    if (r.straightTo(-24, 0)) {
//                        currentState = State.TURN_FACING_FIRST_BALL;
//                    }
//                    break;
//
//                case TURN_FACING_FIRST_BALL:
//                    if (r.turnTo(90)) {
//                        currentState = State.INTAKE;
//                    }
//                    break;
//
//                case INTAKE:
//                    r.Intake(2000);
//                    if (intakeTimer.seconds() > 3) {
//                        r.Intake(0);
//                        currentState = State.FORWARD_INTAKE_1;
//                    }
//
//                case FORWARD_INTAKE_1:
//                    if (r.straightTo(30, 0)) {
//                        currentState = State.BACKWARD_INTAKE_1;
//                    }
//                    break;
//
//                case BACKWARD_INTAKE_1:
//                    if (r.straightTo(-30, 0)) {
//                        currentState = State.CASE_DONE;
//                    }
//                    break;

                case CASE_DONE:
                    break;
            }
        }
    }
}
