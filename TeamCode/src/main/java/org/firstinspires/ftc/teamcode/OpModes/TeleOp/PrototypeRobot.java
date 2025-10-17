package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;

@TeleOp(name = "prototype")
public class PrototypeRobot extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Transfer transfer;
    DriveController driveController;
    FireControl fireControl;
    ElapsedTime time;
    double flag1 = 0;
    double flag2 = 0;
    double shooterVelocity = 15;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        transfer = new Transfer(this);
        time = new ElapsedTime();

        driveController = new DriveController(hardwareMap);
        fireControl = new FireControl(new AprilTagLocalizer(hardwareMap), telemetry);

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {

            intake.simpleDrive(1, gamepad1.right_trigger > 0.25);

            if (gamepad1.right_bumper)
            {
                transfer.driveServo(0.7);
                flag1 = time.seconds() + 1;
            }
            else if (flag1 < time.startTime())
            {
                transfer.driveServo(1);
            }

            if (gamepad1.dpad_up && flag2 < time.seconds())
            {
                shooterVelocity++;
                flag2 = time.seconds() + 1;
            }
            else if (gamepad1.dpad_down  && flag2 < time.seconds())
            {
                shooterVelocity--;
            }


            if (gamepad1.b)
            {
                shooter.driveToVelocity(fireControl.targetMotorVelocity(shooterVelocity));
                if (65 - fireControl.calculateShallowerAngle(shooterVelocity) > 0)
                {
                    hood.driveToAngleTarget(65 - fireControl.calculateShallowerAngle(shooterVelocity));
                }
                else
                {
                    hood.driveToAngleTarget(0);

                }
            }

            if (gamepad1.y)
            {
                shooter.setPower(1);
            }
            else if (gamepad1.x)
            {
                shooter.setPower(0);
            }

            //            driveController.gamepadController(gamepad1);

            telemetry.addData("Hood Projected Angle", 65 - fireControl.calculateShallowerAngle(shooterVelocity));
            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Shooter RPM", (shooter.getVelocity()/28)*60);
            telemetry.update();
        }

    }
}
