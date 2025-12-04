package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

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
import org.firstinspires.ftc.teamcode.FeedbackSystems.PID.PIDController;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v1.Transfer;

@TeleOp(name = "Version_1")
public class Version1 extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Trigger trigger;
    Transfer transfer;

    DriveController driveController;
    FireControl fireControl;
    ElapsedTime time;
    double flag1 = 0;
    double flag2 = 0;
    double initialTargetVelocity = 10;
    double[] shooterParameters;
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    PIDController pid;

    double target_velocity;
    double target_angle;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        trigger = new Trigger(this);
        transfer = new Transfer(this);

        time = new ElapsedTime();
        driveController = new DriveController(hardwareMap);
        fireControl = new FireControl(new AprilTagLocalizer(hardwareMap), telemetry);
        drive = new MecanumDrive(hardwareMap, null);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);




        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
            driveController.gamepadController(gamepad1);

            if (gamepad1.right_bumper || gamepad2.right_bumper)
            {
                trigger.driveServo(0.78);
                flag1 = time.seconds() + 0.5;
            }
            else if (flag1 < time.startTime() && flag1 + 5 > time.seconds())
            {
                trigger.driveServo(1);
                transfer.driveServo(1);
            }
            else if (gamepad1.right_trigger > 0.25)
            {
                transfer.driveServo(1);
            }
            else
            {
                transfer.driveServo(0);
            }


            intake.simpleDrive(1, gamepad1.right_trigger > 0.25);
            if (gamepad1.left_trigger > 0.25)
            {
                intake.setPower(-1.0);
            }

            if (gamepad1.dpad_up && flag2 < time.seconds())
            {
                initialTargetVelocity++;
                flag2 = time.seconds() + 0.25;
            }
            else if (gamepad1.dpad_down  && flag2 < time.seconds())
            {
                initialTargetVelocity--;
                flag2 = time.seconds() + 0.25;
            }

            shooterParameters = fireControl.firingSuite(initialTargetVelocity);
            target_velocity = shooterParameters[1];
            target_angle = shooterParameters[0];

            hood.driveToAngleTarget(target_angle);
            if (gamepad1.b || gamepad2.b)
            {
                shooter.driveToVelocity(target_velocity);
            }
            else if (gamepad1.a || gamepad2.a)
            {
                shooter.setPower(0);
            }

            fireControl.firingSuite(initialTargetVelocity);
            telemetry.addData("Initial Target Velocity", initialTargetVelocity);
            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Motor Velocity: ", shooter.getVelocity());
            telemetry.addData("hood angle", hood.getAngle());
            telemetry.update();
        }

    }
}
