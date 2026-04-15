package org.firstinspires.ftc.teamcode.Outreach.Funyon;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;

@TeleOp(name = "Funyon Outreach", group = "outreach")
//@Disabled
public class FunyonTeleOp extends OpMode
{
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    DriveFunyon mechanicalDriveOutreach;
    FunyonShooter shooter;

    @Override
    public void init()
    {
        mechanicalDriveOutreach = new DriveFunyon(hardwareMap);
        shooter = new FunyonShooter(hardwareMap);
        telemetry.addData("Initialized", " Press start");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    @Override
    public void loop()
    {
        mechanicalDriveOutreach.gamepadController(gamepad1, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger, imu.getRobotYawPitchRollAngles().getYaw(), gamepad1.right_stick_x);
        mechanicalDriveOutreach.driveBaseTelemetry(telemetry);
        shooter.controlMethod(gamepad1);

        if (gamepad1.dpad_up)
        {
            imu.resetYaw();
        }
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw());
    }
}

/** Below is config and code notes **/

/** Drive base expansion hub**/
// lf = 3, REVERSE
// rf = 2, FORWARD
// lb = 1, REVERSE
// rb = 0, FORWARD

/** Firing and intake control hub**/
//  intakeWheel =  0  |||  direction forward
//  intakeMill =  1  |||  direction Reversed
//  shooter = 3 |||
//  servo = 5 (i think) ||| presets should be set
