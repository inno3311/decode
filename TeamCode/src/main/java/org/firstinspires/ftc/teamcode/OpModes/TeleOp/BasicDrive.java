package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;

@TeleOp(name = "Basic Drive", group = "OpMOde")
//@Disabled
public class BasicDrive extends OpMode
{
    DriveController driveController;

    @Override
    public void init()
    {
        driveController = new DriveController(hardwareMap);
    }

    @Override
    public void loop()
    {
        driveController.gamepadController(gamepad1);
        driveController.driveBaseTelemetry(telemetry);
    }
}
