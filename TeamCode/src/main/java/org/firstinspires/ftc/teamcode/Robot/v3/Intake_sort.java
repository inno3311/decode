package org.firstinspires.ftc.teamcode.Robot.v3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.CRServoParent;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class Intake_sort extends MotorParent
{
    public Intake_sort(LinearOpMode opMode)
    {
        super("intakesort", true, false, opMode);
    }

    @Override
    public void setPower(double power)
    {
        super.setPower(power);
    }
}