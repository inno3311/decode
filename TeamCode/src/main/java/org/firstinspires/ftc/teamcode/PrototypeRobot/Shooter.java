package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Parents.MotorParent;

public class Shooter extends MotorParent
{
    public Shooter(LinearOpMode opMode)
    {
        super("shooter", true, false, opMode);
    }

    @Override
    public void toggleDrive(double speed, boolean argument)
    {
        super.toggleDrive(speed, argument);
    }
}
