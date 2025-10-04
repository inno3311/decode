package org.firstinspires.ftc.teamcode.Outreach.Hippo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class HippoStomper extends ServoParent
{
    public HippoStomper(LinearOpMode opMode)
    {
        super("stomper", 0, opMode);
    }

    @Override
    protected void driveServo(double target)
    {
        super.driveServo(target);
    }
}
