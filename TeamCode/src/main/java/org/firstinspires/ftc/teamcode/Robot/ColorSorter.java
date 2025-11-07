package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSorter
{
    private ColorSensor colorSorter;

    public enum color
    {
      green, purple, none
    };

    public ColorSorter(LinearOpMode opMode)
    {
        colorSorter = opMode.hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public color getColor()
    {
        if (colorSorter.red() > 500)
        {
            return color.purple;
        }
        else if (colorSorter.green() > 500)
        {
            return color.green;
        }
        else
        {
            return color.none;
        }
    }

    public color getNextColor(int numberOfBallsScored, double tagNumber)
    {
        color[] pattern;
        if (tagNumber == 21)
        {
            pattern = new color[] {color.green, color.purple, color.purple};
            return pattern[numberOfBallsScored%3];
        }
        else if (tagNumber == 23)
        {
            pattern = new color[] {color.purple, color.purple, color.green};
            return pattern[numberOfBallsScored%3];
        }
        else
        {
            pattern = new color[] {color.purple, color.green, color.purple};
            return pattern[numberOfBallsScored%3];
        }
    }
}
