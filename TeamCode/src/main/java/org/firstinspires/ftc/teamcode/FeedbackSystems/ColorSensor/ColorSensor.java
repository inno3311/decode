package org.firstinspires.ftc.teamcode.FeedbackSystems.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor
{
    NormalizedColorSensor colorSensor;
    public enum DetectedColor
    {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public ColorSensor(HardwareMap hardwareMap)
    {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public NormalizedRGBA getDetectedColor(Telemetry telemetry)
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        float normalizedRed, normalizedGreen, normalizedBlue;
//        normalizedRed = colors.red / colors.alpha;
//        normalizedGreen = colors.green / colors.alpha;
//        normalizedBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red", normalizedRed);
//        telemetry.addData("green", normalizedGreen);
//        telemetry.addData("blue", normalizedBlue);
//        telemetry.update();

        return colors;
    }
}
