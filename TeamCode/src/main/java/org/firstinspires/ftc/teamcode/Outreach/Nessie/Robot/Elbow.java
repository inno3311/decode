package org.firstinspires.ftc.teamcode.Outreach.Nessie.Robot;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Outreach.Nessie.controller.MotorParent;

public class Elbow extends MotorParent
{

    public Elbow(LinearOpMode opMode)
    {
        super("elbow", true, true, opMode);
    }

    @Override
    protected void analogControl(double speedLimit, double input, boolean advanceBreak, boolean slowMode, boolean upperLimit, int lowerLimit, boolean swapBounds)
    {
        super.analogControl(speedLimit, input, advanceBreak, slowMode, upperLimit, lowerLimit, swapBounds);
    }

    @Override
    protected void analogControl(double speedLimit, double input, boolean advanceBreak, boolean slowMode, boolean upperLimit, boolean lowerLimit)
    {
        super.analogControl(speedLimit, input, advanceBreak, slowMode, upperLimit, lowerLimit);
    }

    @Override
    public Action action(int target, double speed)
    {
        return super.action(target, speed);
    }

    @Override
    public void initialize(TouchSensor sensor, int direction, double speed) {super.initialize(sensor, direction, speed);}

    @Override
    protected void automaticEncoderReset(boolean reset) {super.automaticEncoderReset(reset);}

    @Override
    public void resetEncoder() {super.resetEncoder();}

    @Override
    protected int getMotorPosition() {return super.getMotorPosition();}

    public enum Presets
    {
        TOP_CHAMBER,
        BOTTOM_CHAMBER,
        TOP_BUCKET,
        BOTTOM_BUCKET,
        PICKUP_FLOOR,
        PICKUP_WALL,
        PICKUP_SUBMERSIBLE,
        INITIALIZATION
    }

    public void encoderPresets(Presets preset)
    {
        switch (preset)
        {
            case TOP_CHAMBER:
                super.encoderControl(-1350,1);
                break;
            case BOTTOM_CHAMBER:
                super.encoderControl(-925,1);
                break;
            case TOP_BUCKET:
                super.encoderControl(-2400,1);
                break;
            case BOTTOM_BUCKET:
                super.encoderControl(-2430,1);
                break;
            case PICKUP_FLOOR:
                super.encoderControl(0,1);
                break;
            case PICKUP_WALL:
                super.encoderControl(-210, 1);
                break;
            case PICKUP_SUBMERSIBLE:
                super.encoderControl(-1000,1);
                break;
            case INITIALIZATION:
                super.encoderControl(-1351,1);
            default:
                break;
        }
    }

    @Override
    public void telemetry()
    {
        super.telemetry();
    }
}
