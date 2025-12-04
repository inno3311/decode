package org.firstinspires.ftc.teamcode.Robot.CommonFeatures;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class Hood extends ServoParent
{

    public Hood(LinearOpMode opMode)
    {
        super("hood", 25, opMode);
    }

    @Override
    public void driveAngle(boolean iIncrease, boolean iDecrease)
    {
        super.driveAngle(iIncrease, iDecrease);
    }

    @Override
    public void driveToAngleTarget(double angle)
    {
        super.driveToAngleTarget(angle);
    }

    @Override
    public double getAngle()
    {
        return super.getAngle();
    }

}
