package org.firstinspires.ftc.teamcode.Robot.v3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.CRServoParent;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class SorterRight extends CRServoParent
{
    public SorterRight(LinearOpMode opMode)
    {
        super("sortright", opMode);
    }

    @Override
    public void driveServo(double power)
    {
        super.driveServo(power);
    }
}