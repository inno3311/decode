package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Lift;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;

@TeleOp(name = "prototype")
public class PrototypeRobot extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Lift lift;
    Transfer transfer;
    DriveController driveController;
    FireControl fireControl;
    ElapsedTime time;
    double flag1 = 0;
    double flag2 = 0;
    double initialTargetVelocity = 12;
    double[] shooterParameters;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        lift = new Lift(this);
        transfer = new Transfer(this);

        time = new ElapsedTime();
        driveController = new DriveController(hardwareMap);
        fireControl = new FireControl(new AprilTagLocalizer(hardwareMap), telemetry);

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {

            driveController.gamepadController(gamepad1);

            intake.simpleDrive(1, gamepad1.right_trigger > 0.25);
            if (gamepad1.right_trigger > 0.25)
            {
                transfer.driveServo(0);
            }

            if (gamepad1.right_bumper)
            {
                lift.driveServo(0.7);
                flag1 = time.seconds() + 0.25;
            }
            else if (flag1 < time.startTime())
            {
                lift.driveServo(1);
                transfer.driveServo(1);
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


            if (gamepad1.b)
            {
                shooterParameters = fireControl.firingSuite(initialTargetVelocity);
                hood.driveToAngleTarget(shooterParameters[0]);
                shooter.driveToVelocity(shooterParameters[1]);
            }
            else if (gamepad1.a)
            {
                shooter.setPower(0);
            }


            telemetry.addData("Initial Target Velocity", initialTargetVelocity);
            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Shooter RPM", (shooter.getVelocity()/28)*60);
            telemetry.addData("Motor Velocity: ", shooter.getVelocity());
            shooter.currentDraw();
            telemetry.update();
        }

    }
}
