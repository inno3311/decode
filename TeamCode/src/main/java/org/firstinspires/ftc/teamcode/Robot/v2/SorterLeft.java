package org.firstinspires.ftc.teamcode.Robot.v2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class SorterLeft extends ServoParent
{
    public SorterLeft(LinearOpMode opMode)
    {
        super("SorterLeft", 270, opMode);
    }

    @Override
    public void driveServo(double target)
    {
        super.driveServo(target);
    }
}
