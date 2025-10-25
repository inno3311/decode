package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;

@TeleOp(name = "Shooter")
@Disabled
public class ShooterTest extends LinearOpMode
{

    Shooter shooter;
    FireControl fireControl;

    ElapsedTime time;
    double flag1 = 0;
    double flag2 = 0;
    double shooterVelocity = 13;


    @Override
    public void runOpMode() throws InterruptedException
    {
        shooter = new Shooter(this);
        fireControl = new FireControl(null, telemetry);
        time = new ElapsedTime();

        time.startTime();
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.dpad_up && flag2 < time.seconds())
            {
                shooterVelocity += 1;
                flag2 = time.seconds() + 0.25;
            }
            else if (gamepad1.dpad_down && flag2 < time.seconds())
            {
                shooterVelocity -= 1;
                flag2 = time.seconds() + 0.25;
            }

            if (gamepad1.b)
            {
                shooter.driveToVelocity(fireControl.targetMotorVelocity(shooterVelocity));
            }


            if (gamepad1.y)
            {
                shooter.driveToVelocity(0);
            }



            telemetry.addData("ShooterVelocity", shooterVelocity);
//            fireControl.actualVelocity(shooter.getVelocity());
            fireControl.speedTransferPercentage(shooter.getVelocity(), shooterVelocity);
            fireControl.targetMotorVelocity(shooterVelocity);
            telemetry.addData("Motor Velocity: ", shooter.getVelocity());
//            telemetry.addData("Hood Projected Angle", 65 - fireControl.calculateSteeperAngle(shooterVelocity));
//            telemetry.addData("shooter Power", shooter.getPower());
            telemetry.addData("Shooter RPM", (-shooter.getVelocity()/28)*60);
            shooter.currentDraw();
            shooter.telemetry();
            telemetry.update();
        }
    }
}
