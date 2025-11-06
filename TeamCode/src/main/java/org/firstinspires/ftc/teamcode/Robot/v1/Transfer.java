package org.firstinspires.ftc.teamcode.Robot.v1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.CRServoParent;

public class Transfer extends CRServoParent
{
    public Transfer(LinearOpMode opMode)
    {
        super("transfer", opMode);
    }

    @Override
    public void driveServo(double power)
    {
        super.driveServo(power);
    }
}
