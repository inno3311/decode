package org.firstinspires.ftc.teamcode.Robot.v3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.CRServoParent;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class Intake_sort extends MotorParent
{
    public Intake_sort(LinearOpMode opMode)
    {
        super("intakesort", false, false, opMode);
    }

    @Override
    public void simpleDrive(double speed, boolean argument)
    {
        super.simpleDrive(speed, argument);
    }

    @Override
    public void setPower(double power)
    {
        super.setPower(power);
    }

    @Override
    public boolean isBusy()
    {
        return super.isBusy();
    }
}