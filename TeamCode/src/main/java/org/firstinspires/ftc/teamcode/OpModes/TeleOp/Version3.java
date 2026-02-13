package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Flipper;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.teamcode.FeedbackSystems.ColorSensor.ColorSensor;
import org.opencv.core.Mat;

import java.io.File;

@TeleOp(name = "Version3")
public class Version3 extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Flipper trigger;
    SorterLeft sorterLeft;
    SorterRight sorterRight;
    Intake_sort intakeSort;
    Turret turret;
    DriveController driveController;
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    AprilTagLocalizer aprilTagLocalizer;
    FireControl fireControl;
    ElapsedTime time;
    ColorSensor colorSensor;
    double[] shooterParameters;
    int drive_mode = 0;
    double drive_mode_flag = 1;
    boolean team = false; //false = red, true = blue
    double startX = 0;
    double startY = 0;
    double startYaw = 180;

    public enum TurretState
    {
        resetting,
        tracking,
        stopped,
    }

    TurretState state = TurretState.stopped;

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        intake = new Intake(this);
        intakeSort = new Intake_sort(this);
        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(this);
        trigger = new Flipper(this);
        sorterLeft = new SorterLeft(this);
        sorterRight = new SorterRight(this);
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

        // Blackboard our starting position
        drive.localizer.setPose((Pose2d) blackboard.get("Pose"));
        if (drive.localizer.getPose() == null)
        {
            Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startYaw)); // inches, radians
            drive.localizer.setPose(startPose);
        }

        //drive.localizer.setPose(new Pose2d(0,0,0));

        colorSensor = new ColorSensor(hardwareMap);
        NormalizedRGBA colors;
        double hue = 0;


        if (blackboard.get("Alliance") == "BLUE")
        {
            team = true;
        }

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
            colors = colorSensor.getDetectedColor();
            float normalizedRed, normalizedGreen, normalizedBlue;
            normalizedRed = colors.red / colors.alpha;
            normalizedGreen = colors.green / colors.alpha;
            normalizedBlue = colors.blue / colors.alpha;
            if (JavaUtil.colorToHue(colors.toColor()) != 0)
            {
                hue = JavaUtil.colorToHue(colors.toColor());
            }

//            telemetry.addData("red", normalizedRed);
//            telemetry.addData("green", normalizedGreen);
//            telemetry.addData("blue", normalizedBlue);
//            telemetry.addData("hue", hue);

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
            driveController.gamepadController(gamepad1);

//            if (drive_mode % 2 == 1)
//            {
//                driveController.gamepadController(gamepad1);
//            }
//            else
//            {
//                if (gamepad1.dpad_left)
//                {
//                    imu.resetYaw();
//                }
//                centricDrive.drive(
//                        -gamepad1.left_stick_x,
//                        -gamepad1.left_stick_y,
//                        imu.getRobotYawPitchRollAngles().getYaw(),
////                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
//                        gamepad1.right_trigger,
//                        -gamepad1.right_stick_x
//                );
//            }

            if (gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x && drive_mode_flag <= time.seconds())
            {
                drive_mode_flag += time.seconds() + 0.25;
                drive_mode += 1;
            }

            // Roadrunner positioning update
            if ((aprilTagLocalizer.getDetectionID() == 20 || aprilTagLocalizer.getDetectionID() == 24))
            {
//                drive.localizer.setPose(new Pose2d(0,0,imu.getRobotYawPitchRollAngles().getYaw()));
                drive.localizer.setPose(aprilTagLocalizer.getFieldPose());
            }
            drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();

            // Color sensor and intake
            if (gamepad1.right_trigger > 0.1)
            {
                intake.setPower(-1);
                // If green (<200), intake to the LEFT side (looking from the back)
                if (hue <= 200)
                {
                    intakeSort.setPower(1);
                }
                // if purple (>200) intake to the RIGHT side (looking from the back)
                else
                {
                    intakeSort.setPower(-1);
                }
            }
            else if (gamepad1.b)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }

            //if both triggers are pulled, don't try to sort. But bad things can happen.
            if (gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger <= 0.1)
            {
                intakeSort.setPower(0);
            }

            // Load ball
            if (gamepad2.left_bumper && !gamepad1.a)
            {
                sorterLeft.driveServo(-1);
            }
            else
            {
                sorterLeft.driveServo(0);

            }

            if (gamepad2.right_bumper && !gamepad1.a)
            {
                sorterRight.driveServo(1);
            }
            else
            {
                sorterRight.driveServo(0);
            }

            // Fire the ball! Ensure sorters are off.
            if (gamepad1.y || gamepad2.y)
            {
                trigger.driveServo(1);
                sorterRight.driveServo(0);
                sorterLeft.driveServo(0);
            }
            else
            {
                trigger.driveServo(0);
            }


            // Firesuit math
            shooterParameters = fireControl.firingSuite(pose, team);

            shooter.driveToVelocity(shooterParameters[1]);
            hood.driveToAngleTarget(shooterParameters[0]);


            // Turret code
            double aiTurretHeading = 0;
            if (gamepad2.b && !gamepad2.start)
            {
                state = TurretState.resetting;
            }
            else if (gamepad2.x)
            {
                state = TurretState.tracking;
            }
            else if (gamepad2.a)
            {
                state = TurretState.stopped;
            }

            switch (state)
            {
                case resetting:
                    turret.zero(gamepad2);
                    break;
                case tracking:
                    aiTurretHeading = turret.turretAngleToFixedTarget(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()), team, 90);
                    break;
                case stopped:
                    turret.stop();
                    break;
            }

            // Dashboard and telemetry
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            if (!team)
            {
                telemetry.addLine("!!! Team is RED !!!");
            }
            else
            {
                telemetry.addLine("!!! Team is BLUE !!!");
            }

            telemetry.addLine("-----------------------------------------------------");
            telemetry.addData("Bot Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Bot Velocity X", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * Math.cos(Math.toRadians(pose.heading.toDouble())));
            telemetry.addData("Bot Velocity Y", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * Math.sin(Math.toRadians(pose.heading.toDouble())));

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();


        }
    }
}
