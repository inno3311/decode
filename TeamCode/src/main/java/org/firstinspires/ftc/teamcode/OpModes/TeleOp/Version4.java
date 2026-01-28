package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Flipper;
import org.firstinspires.ftc.teamcode.Robot.v4.Trigger;

@TeleOp(name = "Version4")
public class Version4 extends LinearOpMode
{
    Intake intake;
    Trigger trigger;
    DriveController driveController;
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    ElapsedTime time;
    int drive_mode = 0;
    double drive_mode_flag = 1;
    boolean continuousDrive = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        trigger = new Trigger(hardwareMap);

        time = new ElapsedTime();

        driveController = new DriveController(hardwareMap,1,-1,-1);

        drive = new MecanumDrive(hardwareMap, null);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {

            // Drive code
            driveController.gamepadController(gamepad1);


            if (gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x && drive_mode_flag <= time.seconds())
            {
                drive_mode_flag += time.seconds() + 0.25;
                drive_mode += 1;
            }

            // Color sensor and intake
            if (gamepad1.right_trigger > 0.1 || continuousDrive)
            {
                intake.setPower(1);
            }
            else if (gamepad1.b)
            {
                intake.setPower(-1);
            }
            else
            {
                intake.setPower(0);
            }

            if (gamepad1.right_bumper || continuousDrive)
            {
                trigger.setPower(1);
            }
            else
            {
                trigger.setPower(0);
            }

            if (gamepad1.a && !gamepad1.start)
            {
                continuousDrive = true;
            }
            else if (gamepad1.b)
            {
                continuousDrive = false;
            }

            telemetry.update();
        }
    }
}
