package org.firstinspires.ftc.teamcode.Outreach.Nessie.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Outreach.Nessie.controller.MotorParent;

public class Hang extends MotorParent
{

    public Hang(LinearOpMode opMode)
    {
        super("hang", true, true, opMode);
    }

    @Override
    protected void simpleDrive(double speed, boolean argument1, boolean argument2)
    {
        super.simpleDrive(speed, argument1, argument2);
    }

    @Override
    public boolean isBusy()
    {
        return super.isBusy();
    }

    @Override
    public int getMotorPosition()
    {
        return super.getMotorPosition();
    }

    @Override
    public void encoderControl(int target, double speed, double argument)
    {
        super.encoderControl(target, speed, argument);
    }

    public enum Presets
    {
        READY,
        SET,
        GO,
    }

    public void encoderPresets(Presets preset)
    {
        switch (preset)
        {
            case READY:
                super.encoderControl(0,1);
                break;
            case SET:
                super.encoderControl(-6000,1);
                break;
            case GO:
                super.encoderControl(100,1);
                break;
            default:
                break;
        }
    }

    @Override
    protected void telemetry()
    {
        super.telemetry();
    }
}
