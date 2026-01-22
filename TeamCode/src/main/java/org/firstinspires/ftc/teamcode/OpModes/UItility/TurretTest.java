package org.firstinspires.ftc.teamcode.OpModes.UItility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;

@TeleOp(name = "Turret Test", group = "Utility")
public class TurretTest extends OpMode
{
    Turret turret;

    @Override
    public void init()
    {
        turret = new Turret(hardwareMap, telemetry,false);
    }

    @Override
    public void loop()
    {
        turret.tuning(gamepad1);
    }

}
