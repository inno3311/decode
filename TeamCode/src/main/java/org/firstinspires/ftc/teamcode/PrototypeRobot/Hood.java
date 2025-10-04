package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class Hood extends ServoParent
{

    public Hood(LinearOpMode opMode)
    {
        super("hood", 270 * 5/16, opMode);
    }

    @Override
    public void driveAngle(boolean iIncrease, boolean iDecrease)
    {
        super.driveAngle(iIncrease, iDecrease);
    }
}
