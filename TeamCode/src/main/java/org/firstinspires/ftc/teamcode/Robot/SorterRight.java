package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class SorterRight extends ServoParent
{
    public SorterRight( LinearOpMode opMode)
    {
        super("SorterRight", 270, opMode);
    }

    @Override
    public void driveServo(double target)
    {
        super.driveServo(target);
    }
}
