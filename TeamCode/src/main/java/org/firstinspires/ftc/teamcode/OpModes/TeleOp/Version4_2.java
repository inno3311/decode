package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.teamcode.Robot.v4.Trigger;

@TeleOp(name = "Headless Hermit Crab")
public class Version4_2 extends LinearOpMode
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
    double startX = 0;
    double startY = 0;
    double startYaw = 180;
    boolean team = false; //false = red, true = blue
    double turretOffset = 90;
    double flag = 0;
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

        // 🔥 THIS is where you fix vibration blur
        aprilTagLocalizer.setManualCameraExposure(10, 30);

        intake = new Intake(this);
        trigger = new Trigger(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(this);
        turret = new Turret(hardwareMap, telemetry,team);

        time = new ElapsedTime();

        driveController = new DriveController(hardwareMap,-1,1,1);

        fireControl = new FireControl(aprilTagLocalizer, telemetry);

        drive = new MecanumDrive(hardwareMap, null);

//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP
//        );
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        imu.resetYaw();

        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose2d pos = (Pose2d) blackboard.get("Pose");
        if (pos == null)
        {
            pos = new Pose2d(0,0,0);
        }

        drive.localizer.setPose((Pose2d) pos);
        if (drive.localizer.getPose() == null)
        {
            Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startYaw)); // inches, radians
            drive.localizer.setPose(startPose);
        }

        if (blackboard.get("Alliance") == "BLUE")
        {
            team = true;
        }

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {

            // Drive code
            driveController.gamepadController(gamepad1);
//            if (gamepad1.dpad_left)
//            {
//                imu.resetYaw();
//            }
//            centricDrive.drive(
//                    gamepad1.left_stick_x,
//                    gamepad1.left_stick_y,
//                    imu.getRobotYawPitchRollAngles().getYaw(),
////                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
//                    gamepad1.left_trigger,
//                    gamepad1.right_stick_x
//            );


            if (gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x && drive_mode_flag <= time.seconds())
            {
                drive_mode_flag += time.seconds() + 0.25;
                drive_mode += 1;
            }

            // Color sensor and intake
            if (gamepad1.right_trigger > 0.1 || gamepad2.y || gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
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

            if (gamepad2.y || gamepad2.left_trigger > 0.1)
            {
                trigger.setPower(1);
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
//                telemetry.addData("Updated", "Roadrunner reset successful");
            }
            PoseVelocity2d velocity2d = drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();


            // Firesuit math
            shooterParameters = fireControl.firingSuite(pose, velocity2d, team);

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
            else if (gamepad2.a && !gamepad2.start)
            {
                state = Version3.TurretState.stopped;
            }

            if (gamepad2.left_bumper)
            {
                turretOffset -= 2;
                //flag = time.seconds() + 0.3;
            }
            else if (gamepad2.right_bumper)
            {
                turretOffset += 2;
                //flag = time.seconds() + 0.3;
            }

            switch (state)
            {
                case resetting:
                    turret.zero(gamepad2);
                    break;
                case tracking:
//                    aiTurretHeading = turret.turretAngleToFixedTarget(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()), velocity2d, shooterParameters[2], team, turretOffset);
                    aiTurretHeading = turret.turretAngleToFixedTarget(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()), team, turretOffset);
                    break;
                case stopped:
                    turret.stop();
                    break;
            }

            telemetry.addData("Turret Position", turret.getPosition());
            telemetry.addData("TurretOffset", turretOffset);
            telemetry.addData("Target Shooter Velocity:", shooterParameters[1]);
            telemetry.addData("Target Hood Angle:", 90 - shooterParameters[0]);
            telemetry.addLine("==========================================");
            telemetry.addData("Robot linear velocity X", velocity2d.linearVel.x);
            telemetry.addData("Robot linear velocity Y", velocity2d.linearVel.y);
            telemetry.addData("Robot angular velocity", velocity2d.angVel);
            telemetry.addData("Robot Heading", Math.toDegrees(pose.heading.toDouble()));

            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);
            packet.put("Robot linear velocity X", velocity2d.linearVel.x);
            packet.put("Robot linear velocity Y", velocity2d.linearVel.y);
            packet.put("Robot angular velocity", velocity2d.angVel);

        }
    }
}
