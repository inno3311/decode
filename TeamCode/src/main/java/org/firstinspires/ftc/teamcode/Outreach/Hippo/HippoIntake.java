package org.firstinspires.ftc.teamcode.Outreach.Hippo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class HippoIntake extends MotorParent
{
    // + speed is intaking
    public HippoIntake(LinearOpMode opMode)
    {
        super("intake", true, false, opMode);
    }

    @Override
    protected void motorBreak()
    {
        super.motorBreak();
    }

    @Override
    protected void simpleDrive(double speed, boolean argument1, boolean argument2)
    {
        super.simpleDrive(speed, argument1, argument2);
    }

    @Override
    protected double getPower()
    {
        return super.getPower();
    }
}
