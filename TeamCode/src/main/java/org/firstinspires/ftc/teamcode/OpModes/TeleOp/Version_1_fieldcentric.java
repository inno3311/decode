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

@TeleOp(name = "Version_1_Centric")
public class Version_1_fieldcentric extends LinearOpMode
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
    double initialTargetVelocity = 12;
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
            if (gamepad1.dpad_left)
            {
                imu.resetYaw();
            }
            centricDrive.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                imu.getRobotYawPitchRollAngles().getYaw(),
//                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                gamepad1.right_trigger,
                gamepad1.right_stick_x
            );

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
                target_velocity = 1100;
                target_angle = 35;
            }


            if (gamepad2.a)
            {
                // Far
                telemetry.addData("__location", "far");
                target_velocity = 1100;
                target_angle = 23;
            }

            if (gamepad2.dpad_up)
            {
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




//            if (gamepad1.x)
//            {
//                shooterParameters = fireControl.firingSuite(initialTargetVelocity);
                telemetry.addData("__Hood target Angle", target_angle);
                telemetry.addData("__Target Velocity", target_velocity);
//            }

            //fireControl.firingSuite(12);
            telemetry.addData("Initial Target Velocity", initialTargetVelocity);
            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Shooter RPM", (shooter.getVelocity()/28)*60);
            telemetry.addData("Motor Velocity: ", shooter.getVelocity());
//            telemetry.addData("Trigger pos", trigger.getPosition());
            shooter.currentDraw();
            telemetry.update();
        }

    }



}
