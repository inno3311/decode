package org.firstinspires.ftc.teamcode.Robot.CommonFeatures;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class Hood extends ServoParent
{

    public Hood(LinearOpMode opMode)
    {
        super("hood", 36, opMode); // 37-38, including 38.5, and 40 do not work well; 39 worked best
//        super("hood", 25, opMode); Original
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

    @Override
    public double getPosition()
    {
        return super.getPosition();
    }
}
