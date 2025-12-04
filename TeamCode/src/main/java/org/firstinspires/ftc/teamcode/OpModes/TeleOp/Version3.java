package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
//import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
//import org.firstinspires.ftc.teamcode.Misc.FireControl;
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
    AprilTagLocalizer aprilTagLocalizer;
    PIDController pid;
    FireControl fireControl;
    ElapsedTime time;
    double initialTargetVelocity = 12;
    double[] shooterParameters;
    double obeliskTag = 22;
    int numberOfBallsScored = 0;
    int drive_mode = 0;
    double drive_mode_flag = 1;
    double target_velocity;
    double target_angle;


    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        intake = new Intake(this);
        intakeSort = new Intake_sort(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        trigger = new Trigger(this);
        sorterLeft = new SorterLeft(this);
        sorterRight = new SorterRight(this);
        turret = new Turret(aprilTagLocalizer, this);


        time = new ElapsedTime();
        driveController = new DriveController(hardwareMap);
//        fireControl = new FireControl(aprilTagLocalizer, telemetry);
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

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
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
            if (gamepad1.a)
            {
                trigger.driveServo(1);
                sorterRight.driveServo(0);
                sorterLeft.driveServo(0);
            }
            else
            {
                trigger.driveServo(0);
            }

            if (gamepad2.x)
            {
                shooterParameters = fireControl.firingSuite(initialTargetVelocity);
                target_velocity = shooterParameters[1];
                target_angle = shooterParameters[0];
            }

            if (gamepad2.y)
            {
                // Obelisk
                telemetry.addData("__location", "obelisk");
                target_velocity = 1150;
                target_angle = 8;
            }


            if (gamepad2.a)
            {
                // Far
                telemetry.addData("__location", "far");
                target_velocity = 1500;
                target_angle = 8;
            }

            if (gamepad2.dpad_up)
            {
                //trigger.driveServo(1);
                target_velocity += 10;
            }
            if (gamepad2.dpad_down)
            {
                target_velocity -= 10;
            }
            if (gamepad2.dpad_left)
            {
                target_angle -= 1;
            }
            if (gamepad2.dpad_right)
            {
                target_angle += 1;
            }

            hood.driveToAngleTarget(target_angle);
            if (gamepad1.b || gamepad2.b)
            {
                shooter.driveToVelocity(target_velocity);
            }
            else if (gamepad1.a || gamepad2.a)
            {
                shooter.setPower(0);
            }

            turret.trackTarget(gamepad2);
            turret.telemetry(telemetry);

            telemetry.update();
        }

    }
}
