package org.firstinspires.ftc.teamcode.Robot.v3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.CRServoParent;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class SorterLeft extends CRServoParent
{
    public SorterLeft(LinearOpMode opMode)
    {
        super("sortleft", opMode);
    }

    @Override
    public void driveServo(double power)
    {
        super.driveServo(power);
    }
}