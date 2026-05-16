package org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DECODE.V2.ColorSensor.NormalizeColorSensor;

public class Intake {
    DcMotor Intake1, Intake2;
    private NormalizeColorSensor colorSensor1, colorSensor2, colorSensor3;
    private NormalizeColorSensor.detectColors dColor1, dColor2, dColor3;

    private enum IntakeState {
        INTAKE,
        TRANSFER,
        LAUNCHING,
        REVERSED,
        STOP,
    }

    IntakeState currentIntake = IntakeState.STOP;

    public Intake(HardwareMap hardwareMap) {
        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");

        colorSensor1 = new NormalizeColorSensor(hardwareMap, "color1");
        colorSensor2 = new NormalizeColorSensor(hardwareMap, "color2");
        colorSensor3 = new NormalizeColorSensor(hardwareMap, "color3");
    }


    public void intake(double FORWARD, double REVERSED, boolean isReadyToShoot, Telemetry telemetry) {
        dColor1 = colorSensor1.getDetectedColor(telemetry);
        dColor2 = colorSensor2.getDetectedColor(telemetry);
        dColor3 = colorSensor3.getDetectedColor(telemetry);

        boolean front = isRing(dColor1);
        boolean middle = isRing(dColor2);
        boolean atGate = isRing(dColor3);

//        MANUAL SYSTEM
        if (FORWARD > 0.1) {
            Intake1.setPower(1.0);
            Intake2.setPower(0.6);
            currentIntake = IntakeState.INTAKE;
        }
        else if (REVERSED > 0.1) {
            Intake1.setPower(-1);
            Intake2.setPower(-1);
            currentIntake = IntakeState.REVERSED;
        }

//        AUTO SYSTEM

//        AUTO STOP klo semua penuh
        if (front && middle && atGate) { Intake1.setPower(0); Intake2.setPower(0);}

//        Launching
        if (isReadyToShoot && atGate) {
            Intake2.setPower(1.0);
            Intake1.setPower(0.8);
            currentIntake = IntakeState.LAUNCHING;
        }

        //        Tranfer dari tengah ke tengah
        if (front && !middle) {
            Intake1.setPower(1.0);
            Intake2.setPower(0.6);
            currentIntake = IntakeState.TRANSFER;
        }

//        Transfer bola dari tengah kebelakang
        if (middle && !atGate) {
            Intake1.setPower(1.0);
            Intake2.setPower(0.6);
            currentIntake = IntakeState.TRANSFER;
        }

        Intake1.setPower(0);
        Intake2.setPower(0);
        currentIntake = IntakeState.STOP;
    }

    private boolean isRing(NormalizeColorSensor.detectColors color) {
        return color == NormalizeColorSensor.detectColors.PURPLE ||
                color == NormalizeColorSensor.detectColors.GREEN;
    }
}
