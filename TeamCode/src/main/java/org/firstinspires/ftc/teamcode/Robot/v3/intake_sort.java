package org.firstinspires.ftc.teamcode.Robot.v3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.CRServoParent;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class intake_sort extends CRServoParent
{
    public intake_sort(LinearOpMode opMode)
    {
        super("intakesort", opMode);
    }

    @Override
    public void driveServo(double power)
    {
        super.driveServo(power);
    }
}