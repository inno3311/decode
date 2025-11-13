package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class Hood extends ServoParent
{

    public Hood(LinearOpMode opMode)
    {
        super("hood", 85, opMode); //270 * (10/32)
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
    public double getPosition()
    {
        return super.getPosition();
    }
}
