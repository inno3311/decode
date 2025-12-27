package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.FeedbackSystems.PID.PIDController;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;

@TeleOp(name = "Version3")
public class Version3 extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Trigger trigger;
    SorterLeft sorterLeft;
    SorterRight sorterRight;
    Intake_sort intakeSort;
    Turret turret;
    DriveController driveController;
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    IMU turretFacing;
    AprilTagLocalizer aprilTagLocalizer;
    FireControl fireControl;
    ElapsedTime time;
    double initialTargetVelocity = 10;
    double[] shooterParameters;
    double obeliskTag = 22;
    int numberOfBallsScored = 0;
    int drive_mode = 0;
    double drive_mode_flag = 1;
    double target_velocity = 0;
    double target_angle = 0;
    double flag2 = 0;
    boolean team = false; //false = red, true = blue

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        intake = new Intake(this);
        intakeSort = new Intake_sort(this);
        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(this);
        trigger = new Trigger(this);
        sorterLeft = new SorterLeft(this);
        sorterRight = new SorterRight(this);
        turret = new Turret(hardwareMap, telemetry);

        time = new ElapsedTime();
        //driveController = new DriveController(hardwareMap);
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

        turretFacing = hardwareMap.get(IMU.class, "imu");
        turretFacing.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180)); // inches, radians
        drive.localizer.setPose(startPose);
        //drive.localizer.setPose(new Pose2d(0,0,0));

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
            if (gamepad2.left_stick_button)
            {
                team = false;
            }
            else if (gamepad2.right_stick_button)
            {
                team = true;
            }

            if (drive_mode % 2 == 1)
            {
                driveController.gamepadController(gamepad1);
            }
            else
            {
                if (gamepad1.dpad_left)
                {
                    imu.resetYaw();
                }
                centricDrive.drive(
                        -gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        imu.getRobotYawPitchRollAngles().getYaw(),
//                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                        gamepad1.right_trigger,
                        -gamepad1.right_stick_x
                );
            }

            if (gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x && drive_mode_flag <= time.seconds())
            {
                drive_mode_flag += time.seconds() + 0.25;
                drive_mode += 1;
            }


            if (gamepad1.right_trigger > 0.1)
            {
                intake.setPower(-.8);
                intakeSort.setPower(1);
            }
            else if (gamepad1.left_trigger > 0.1)
            {
                intake.setPower(-.8);
                intakeSort.setPower(-1);
            }
            else if (gamepad1.b)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }

            //if both triggers are pulled, don't try to sort.  But bad things can happen.
            if (gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger <= 0.1)
            {
                intakeSort.setPower(0);
            }

            if (gamepad1.left_bumper && !gamepad1.a)
            {
                sorterLeft.driveServo(-1);
            }
            else
            {
                sorterLeft.driveServo(0);

            }

            if (gamepad1.right_bumper && !gamepad1.a)
            {
                sorterRight.driveServo(1);
            }
            else
            {
                sorterRight.driveServo(0);
            }

            //Fire the ball!  Ensure sorters are off.
            if (gamepad1.y)
            {
                trigger.driveServo(1);
                sorterRight.driveServo(0);
                sorterLeft.driveServo(0);
            }
            else
            {
                trigger.driveServo(0);
            }

            if (gamepad2.left_bumper)
            {
                initialTargetVelocity = 10.5;
            }
            else if (gamepad2.left_trigger > 0.5)
            {
                initialTargetVelocity = 9.5;
            }

            if (gamepad2.dpad_up && flag2 < time.seconds())
            {
                initialTargetVelocity += 0.5;
                flag2 = time.seconds() + 0.25;
            }
            else if (gamepad2.dpad_down  && flag2 < time.seconds())
            {
                initialTargetVelocity -= 0.5;
                flag2 = time.seconds() + 0.25;
            }

            // Roadrunner positioning update
            if (aprilTagLocalizer.getDetectionID() != -1)
            {
//                drive.localizer.setPose(aprilTagLocalizer.getFieldPose());
            }

            drive.localizer.update();
            Pose2d pose2d = drive.localizer.getPose();


            shooterParameters = fireControl.firingSuite(initialTargetVelocity, pose2d, team);
            target_velocity = shooterParameters[1];
            target_angle = shooterParameters[0];

            hood.driveToAngleTarget(target_angle);

            if (gamepad1.b || gamepad2.b)
            {
                shooter.driveToVelocity(target_velocity);
            }
            else if (gamepad1.a || gamepad2.a)
            {
                shooter.driveToVelocity(0);
            }


            turret.trackGoal(turretFacing.getRobotYawPitchRollAngles().getYaw(), pose2d, team, gamepad2);
            telemetry.addData("Turret Facing", turretFacing.getRobotYawPitchRollAngles().getYaw());


            Pose2d pose = drive.localizer.getPose();


            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

//            telemetry.addData("x (in)", pose.position.x);
//            telemetry.addData("y (in)", pose.position.y);
//            telemetry.addData("heading (deg)",
//                    Math.toDegrees(pose.heading.toDouble()));
//
//            telemetry.addData("Initial Target Velocity", initialTargetVelocity);
//            telemetry.addData("Hood angle", hood.getAngle());
//            telemetry.addData("Hood Position", hood.getPosition());
//            telemetry.addLine("-----------------------------------------------------");



            fieldOverlay.setStroke("#3F51B5"); // Blue
            fieldOverlay.strokeCircle(pose.position.x, pose.position.y, 3); // x, y, radius
            fieldOverlay.strokeLine(
                    pose.position.x,
                    pose.position.y, pose.position.x + 9 * Math.cos(pose.heading.toDouble()),
                    pose.position.y + 9 * Math.sin(pose.heading.toDouble()));

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

        }

    }
}
