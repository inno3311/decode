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
    double initialTargetVelocity = 11;
    double[] shooterParameters;
    double obeliskTag = 22;
    int numberOfBallsScored = 0;
    int drive_mode = 0;
    double drive_mode_flag = 1;
    double flag2 = 0;



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

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {

            // Drive Code
//==================================================================================================================================================================================================
            if (drive_mode % 2 == 1)
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
                        -gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        imu.getRobotYawPitchRollAngles().getYaw(),
//                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                        gamepad1.right_trigger,
                        -gamepad1.right_stick_x
                );
            }

            // Change drive type
            if (gamepad1.back && drive_mode_flag <= time.seconds())
            {
                drive_mode_flag += time.seconds() + 0.25;
                drive_mode += 1;
            }

            // Intake Code
//==================================================================================================================================================================================================
            if (gamepad1.right_trigger > 0.1)
            {
                intake.setPower(-1);
                intakeSort.setPower(-1);
            }
            else if (gamepad1.left_trigger > 0.1)
            {
                intake.setPower(-1);
                intakeSort.setPower(1);
            }
            else if (gamepad1.b)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
                intakeSort.setPower(0);
            }

            //Load Ball Code
//==================================================================================================================================================================================================
            if (gamepad1.dpad_left || gamepad2.dpad_left)
            {
                sorterLeft.driveServo(-1);
            }
            else
            {
                sorterLeft.driveServo(0);
            }

            if (gamepad1.dpad_right || gamepad2.dpad_right)
            {
                sorterRight.driveServo(1);
            }
            else
            {
                sorterRight.driveServo(0);
            }

            // Trigger Code
//==================================================================================================================================================================================================
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

            // Target velocity code
//==================================================================================================================================================================================================
            if (gamepad2.dpad_up && flag2 < time.seconds())
            {
                initialTargetVelocity++;
                flag2 = time.seconds() + 0.25;
            }
            else if (gamepad2.dpad_down  && flag2 < time.seconds())
            {
                initialTargetVelocity--;
                flag2 = time.seconds() + 0.25;
            }

            //Shooter and hood code
//==================================================================================================================================================================================================
            shooterParameters = fireControl.firingSuite(initialTargetVelocity);

            hood.driveToAngleTarget(90-shooterParameters[0]);

            if (gamepad1.x || gamepad2.x)
            {
                shooter.driveToVelocity(shooterParameters[1]);
            }
            else if (gamepad1.a || gamepad2.a)
            {
                shooter.setPower(0);
            }

            // Turret Code
//            turret.trackTarget(gamepad2);
//            turret.telemetry(telemetry);

            // Telemetry
//==================================================================================================================================================================================================
            telemetry.addData("Initial Target Velocity", initialTargetVelocity);
            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Motor Velocity:", shooter.getVelocity());
            telemetry.addData("hood angle", hood.getAngle());
            telemetry.update();
        }

    }
}
