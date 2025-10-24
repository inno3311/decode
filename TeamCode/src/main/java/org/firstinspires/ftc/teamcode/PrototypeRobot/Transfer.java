package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class Transfer extends ServoParent
{
    public Transfer(LinearOpMode opMode)
    {
        super("lift", 0, opMode);
    }

    @Override
    public void driveServo(double target)
    {
        super.driveServo(target);
    }

    @Override
    public double getPosition()
    {
        return super.getPosition();
    }
}
