package org.firstinspires.ftc.teamcode.OpModes.UItility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;

@Disabled
@TeleOp(name = "Shooter", group = "Utility")
public class ShooterTest extends OpMode
{
    Shooter shooter;
    Turret turret;
    FireControl fireControl;
    DriveController driveController;
    double tagetVelocity = 10;

    FtcDashboard dashboard;
    TelemetryPacket packet;


    @Override
    public void init()
    {
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap,telemetry,false);
        fireControl = new FireControl(new AprilTagLocalizer(hardwareMap), telemetry);
        driveController = new DriveController(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Starting testing");
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop()
    {
        driveController.gamepadController(gamepad1);

//        shooter.tuning(fireControl.targetMotorVelocity(tagetVelocity), gamepad1);
        if (gamepad1.left_bumper)
        {
            tagetVelocity = 10;
        }
        else if (gamepad1.right_bumper)
        {
            tagetVelocity = 8;
        }
        else if (gamepad1.a)
        {
            tagetVelocity = 0;
        }

        //packet.put("targetVel", tagetVelocity);
        //dashboard.sendTelemetryPacket(packet);
        
    }






}
