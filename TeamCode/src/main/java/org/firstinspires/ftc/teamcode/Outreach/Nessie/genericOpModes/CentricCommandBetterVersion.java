package org.firstinspires.ftc.teamcode.Outreach.Nessie.genericOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Nessie.IMU.IMUControl;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.fieldCentric.CentricDriveTheBetterVersion;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.fieldCentric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.roadrunner.MecanumDrive;

@TeleOp(name = "Centric Command", group = "FieldCentric")
@Disabled
public class CentricCommandBetterVersion extends OpMode
{
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDriveTheBetterVersion centricDriveTheBetterVersion;
    IMUControl imu;

    @Override
    public void init()
    {
        drive = new MecanumDrive(hardwareMap, null);
        imu = new IMUControl(hardwareMap, telemetry);
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDriveTheBetterVersion = new CentricDriveTheBetterVersion(drive, telemetry, new ElapsedTime());
    }

    @Override
    public void loop()
    {
        centricDriveTheBetterVersion.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, imu.getAngle(), turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2), new Gamepad());

    }
}
