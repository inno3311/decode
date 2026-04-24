package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.teamcode.Roadrunner.Drawing;
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

    boolean isFlyWheelDisabled = false;
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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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

            // Set team for turret tracking and firesuit
            if (gamepad2.right_stick_button)
            {
                team = false; // Red
            }
            else if (gamepad2.left_stick_button)
            {
                team = true; // Blue
            }

            // Drive code
            if(drive_mode%2 == 1)
            {
                driveController.gamepadController(gamepad1);
            }
            else
            {
                if (gamepad1.dpad_up)
                {
                    imu.resetYaw();
                }
                centricDrive.drive(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    imu.getRobotYawPitchRollAngles().getYaw(),
//                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                    gamepad2.left_trigger,
                    gamepad1.right_stick_x
                );
            }

            if (gamepad1.back && drive_mode_flag <= time.seconds())
            {
                drive_mode_flag += time.seconds() + 0.25;
                drive_mode += 1;
            }

            // Color sensor and intake
            if (gamepad1.right_trigger > 0.1 || gamepad1.y)
            {
                intake.setPower(-1);
            }
            else if (gamepad1.left_trigger > 0.1 )
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }

            if (gamepad1.y)
            {
                trigger.setPower(1);
            }
            else
            {
                trigger.setPower(0);
            }

            //if (gamepad1.aWasPressed())
            if (gamepad1.dpadLeftWasPressed())
            {
                //shooter.setDisabled(false);
                if (isFlyWheelDisabled == false)
                {
                    isFlyWheelDisabled = true;
                }
                else
                {
                    isFlyWheelDisabled = false;
                }
                shooter.setDisabled(isFlyWheelDisabled);
            }
//            if ((gamepad1.start))
//            {
//                shooter.setDisabled(true);
//            }
//            else
//            {
//                shooter.setDisabled(false);
//            }
            // Roadrunner positioning update
            // Must now use Start button to reset.
            if ((aprilTagLocalizer.getDetectionID() == 20 || aprilTagLocalizer.getDetectionID() == 24) && (gamepad1.start))
            {
//                drive.localizer.setPose(new Pose2d(0,0,imu.getRobotYawPitchRollAngles().getYaw()));
                drive.localizer.setPose(aprilTagLocalizer.getFieldPose());
            }
            PoseVelocity2d velocity2d = drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();


            // Firesuit math
            shooterParameters = fireControl.firingSuite(pose, velocity2d, team);

            //jrm comment following like to disable flywheel.
            shooter.driveToVelocity(shooterParameters[1]);
            hood.driveToAngleTarget(shooterParameters[0]);

            // Turret code
            double aiTurretHeading = 0;
            if (gamepad1.b && !gamepad2.start)
            {
                state = Version3.TurretState.resetting;
            }
            else if (gamepad1.x)
            {
                state = Version3.TurretState.tracking;
            }
            else if (gamepad1.a && !gamepad2.start)
            {
                state = Version3.TurretState.stopped;
            }

            if (gamepad1.left_bumper)
            {
                turretOffset -= 1;
                //flag = time.seconds() + 0.3;
            }
            else if (gamepad1.right_bumper)
            {
                turretOffset += 1;
                //flag = time.seconds() + 0.3;
            }
            else if (gamepad1.dpad_down)
            {
                turretOffset = 90;
            }

            switch (state)
            {
                case resetting:
                    turret.zero(gamepad1);
                    break;
                case tracking:
//                    aiTurretHeading = turret.turretAngleToFixedTarget(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()), velocity2d, shooterParameters[2], team, turretOffset);
                    aiTurretHeading = turret.turretAngleToFixedTarget(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()), team, turretOffset);
                    break;
                case stopped:
                    turret.stop();
                    break;
            }

            telemetry.addData("01_Turret Position", turret.getPosition());
            telemetry.addData("02_TurretOffset", turretOffset);
            telemetry.addData("03_TurretHeading", aiTurretHeading);
            telemetry.addData("04_Target Shooter Velocity:", shooterParameters[1]);
            telemetry.addData("05_Target Hood Angle:", 90 - shooterParameters[0]);
            telemetry.addLine("05_==========================================");
//            telemetry.addData("06_Robot linear velocity X", velocity2d.linearVel.x);
//            telemetry.addData("07_Robot linear velocity Y", velocity2d.linearVel.y);
//            telemetry.addData("08_Robot angular velocity", velocity2d.angVel);
            telemetry.addData("09_Robot Heading", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("10_Drive Mode", drive_mode%2);




            telemetry.update();


            double length = 6; // length of turret line in inches (adjust for visibility)
            double turretX = pose.position.x + length * Math.cos(Math.toRadians(aiTurretHeading));
            double turretY = pose.position.y + length * Math.sin(Math.toRadians(aiTurretHeading));



            TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
//            packet.put("Robot linear velocity X", velocity2d.linearVel.x);
//            packet.put("Robot linear velocity Y", velocity2d.linearVel.y);
//            packet.put("Robot angular velocity", velocity2d.angVel);
            Drawing.drawRobot(c, pose);

            // draw turret direction
            c.setStroke("red");
            c.strokeLine(pose.position.x, pose.position.y, turretX, turretY);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
