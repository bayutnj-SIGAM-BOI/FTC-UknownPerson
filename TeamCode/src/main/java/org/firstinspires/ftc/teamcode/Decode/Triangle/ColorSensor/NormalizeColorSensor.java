package org.firstinspires.ftc.teamcode.Decode.Triangle.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class NormalizeColorSensor {
    NormalizedColorSensor colorSensor;

    public enum detectColors {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public NormalizeColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(4);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

    }

    public detectColors getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float NormalizePurple, NormalizeGreen;
        NormalizePurple = colors.blue / colors.alpha;
        NormalizeGreen = colors.red / colors.alpha;

        if (NormalizePurple > 0.3 && NormalizeGreen <= 0.3) {
            return detectColors.PURPLE;
        } else if (NormalizePurple < 0.3 && NormalizeGreen >= 0.3) {
            return detectColors.GREEN;
        }
        return detectColors.UNKNOWN;
    }
}
