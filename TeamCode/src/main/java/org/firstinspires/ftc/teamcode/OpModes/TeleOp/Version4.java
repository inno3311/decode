package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Flipper;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.teamcode.Robot.v4.Trigger;

@TeleOp(name = "Version4")
public class Version4 extends LinearOpMode
{
    Intake intake;
    Trigger trigger;
    Shooter shooter;
    Hood hood;
    Turret turret;
    DriveController driveController;
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    AprilTagLocalizer aprilTagLocalizer;
    IMU imu;
    ElapsedTime time;
    FireControl fireControl;
    double[] shooterParameters;
    int drive_mode = 0;
    double drive_mode_flag = 1;
    boolean continuousDrive = false;
    boolean team = false; //false = red, true = blue
    public enum TurretState
    {
        resetting,
        tracking,
        stopped,
    }

    Version3.TurretState state = Version3.TurretState.stopped;

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        intake = new Intake(this);
        trigger = new Trigger(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(this);
        turret = new Turret(hardwareMap, telemetry,team);

        time = new ElapsedTime();

        driveController = new DriveController(hardwareMap,-1,1,1);

        fireControl = new FireControl(aprilTagLocalizer, telemetry);

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

        //jrm   temp code
        //drive.localizer.setPose(new Pose2d(0,0,0));

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
                intake.setPower(-1);
            }
            else if (gamepad1.b)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }

            if (gamepad1.y || continuousDrive)
            {
                trigger.setPower(-1);
            }
            else
            {
                trigger.setPower(0);
            }

            // Roadrunner positioning update
            if ((aprilTagLocalizer.getDetectionID() == 20 || aprilTagLocalizer.getDetectionID() == 24))
            {
//                drive.localizer.setPose(new Pose2d(0,0,imu.getRobotYawPitchRollAngles().getYaw()));
                drive.localizer.setPose(aprilTagLocalizer.getFieldPose());
            }
            drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();


//            if (pose == null)
//                continue;

            // Firesuit math
            shooterParameters = fireControl.firingSuite(pose, team);

            shooter.driveToVelocity(shooterParameters[1]);
            hood.driveToAngleTarget(shooterParameters[0]);

            // Turret code
            double aiTurretHeading = 0;
            if (gamepad2.b && !gamepad2.start)
            {
                state = Version3.TurretState.resetting;
            }
            else if (gamepad2.x)
            {
                state = Version3.TurretState.tracking;
            }
            else if (gamepad2.a)
            {
                state = Version3.TurretState.stopped;
            }

            switch (state)
            {
                case resetting:
                    turret.zero(gamepad2);
                    break;
                case tracking:
                    aiTurretHeading = turret.turretAngleToFixedTarget(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()), team);
                    break;
                case stopped:
                    turret.stop();
                    break;
            }


            telemetry.addData("Bot  X", pose.position.x);
            telemetry.addData("Bot  Y", pose.position.y);
            telemetry.addData("Bot  Heading", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Turret  Heading", Math.toDegrees(aiTurretHeading));

            telemetry.addData("Shooter Vel:",shooterParameters[1]);
            telemetry.addData("Hood Angle:",shooterParameters[0]);


            telemetry.update();
        }
    }
}
