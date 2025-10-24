package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class Intake extends MotorParent
{
    public Intake(LinearOpMode opMode)
    {
        super("intake", false, false, opMode);
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
