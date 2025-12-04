package org.firstinspires.ftc.teamcode.Outreach.Hippo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class HippoShooter extends MotorParent
{
    // + speed is shooting
    public HippoShooter(LinearOpMode opMode)
    {
        super("shooter", true, false, opMode);
    }

    @Override
    protected void setPower(double speed)
    {
        super.setPower(speed);
    }
}
