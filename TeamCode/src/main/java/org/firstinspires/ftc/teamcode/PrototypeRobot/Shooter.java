package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class Shooter extends MotorParent
{
    public Shooter(LinearOpMode opMode)
    {
        super("shooter", false, true, opMode);
    }

    @Override
    public void toggleDrive(double speed, boolean argument)
    {
        super.toggleDrive(speed, argument);
    }

    @Override
    public void driveToVelocity(double targetVelocity)
    {
        super.driveToVelocity(targetVelocity);
    }

    @Override
    public double getVelocity()
    {
        return super.getVelocity();
    }

    @Override
    public double getPower()
    {
        return super.getPower();
    }

    @Override
    public void setPower(double power)
    {
        super.setPower(power);
    }

    @Override
    public void telemetry()
    {
        super.telemetry();
    }

}
