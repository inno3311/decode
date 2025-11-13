package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDriveTheBetterVersion;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
//import org.firstinspires.ftc.teamcode.FeedbackSystems.IMU.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;

@TeleOp(name = "Centric Command", group = "FieldCentric")
@Disabled
public class CentricCommandBetterVersion extends OpMode
{
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDriveTheBetterVersion centricDriveTheBetterVersion;
    IMU imu;

    @Override
    public void init()
    {
        drive = new MecanumDrive(hardwareMap, null);
        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        drive = new MecanumDrive(hardwareMap, null);
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDriveTheBetterVersion = new CentricDriveTheBetterVersion(drive, telemetry, new ElapsedTime());
    }

    @Override
    public void loop()
    {
        centricDriveTheBetterVersion.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                imu.getRobotYawPitchRollAngles().getYaw(),
//                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                gamepad1.right_stick_x,
                new Gamepad()
        );
    }
}
