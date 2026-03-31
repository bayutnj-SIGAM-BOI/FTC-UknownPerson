package org.firstinspires.ftc.teamcode.AdvanceCode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;

@Autonomous
public class AutoStateMachineDecode extends LinearOpMode {
    Config autoConf = new Config();
    ElapsedTime ShootingTimer = new ElapsedTime();
    ElapsedTime IntakeTimer = new ElapsedTime();

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
        SPLINE_1,
        STOP,
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autoConf.initialize(hardwareMap, false, false, true, false);

        waitForStart();
        telemetry.addData("Initialize", "Success!, Ready TOGOOOO!!!");
        State currentState = State.FACE_GOAL_1;
        while (opModeIsActive()) {
            switch (currentState) {

            }
        }
    }
}
