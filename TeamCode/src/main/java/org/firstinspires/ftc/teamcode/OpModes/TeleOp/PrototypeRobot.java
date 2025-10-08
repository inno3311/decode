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
    double flag = 0;
    double flag1 = 0;
    double shooterVelocity = 0;

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

            if (flag1 < time.seconds())
            {
                shooter.toggleDrive(1, gamepad1.a && !gamepad1.start);
                if (gamepad1.b)
                {
                    flag1 = time.seconds() + 0.5;
                }
            }

            hood.driveAngle(gamepad1.dpad_up, gamepad1.dpad_down);

            if (gamepad1.right_bumper)
            {
                transfer.driveServo(0.7);
                flag = time.seconds() + 1;
            }
            else if (flag < time.startTime())
            {
                transfer.driveServo(1);
            }

//            driveController.gamepadController(gamepad1);


            if (gamepad1.b)
            {
                if (fireControl.calculateAngle(15)-25 > 0)
                {
                    hood.driveToAngleTarget(fireControl.calculateAngle(15) + 25);
                }
                else
                {
                    hood.driveToAngleTarget(0);

                }
                shooterVelocity = 15;
            }

            fireControl.calculateAngle(15);
            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();
        }

    }
}
