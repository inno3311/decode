package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.v2.ColorSorter;
import org.firstinspires.ftc.teamcode.Robot.v2.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v2.SorterRight;

@TeleOp(name = "Version2")
public class Version2 extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Trigger trigger;
    SorterLeft sorterLeft;
    SorterRight sorterRight;
    ColorSorter colorSorter;
    DriveController driveController;
    FireControl fireControl;
    ElapsedTime time;
    double flag1 = 0;
    double flag2 = 0;
    double flag3 = 0;
    double flag4 = 0;
    double flag5 = 0;
    boolean sort = false;
    double initialTargetVelocity = 12;
    double[] shooterParameters;
    double obeliskTag = 22;
    int numberOfBallsScored = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        trigger = new Trigger(this);
        sorterLeft = new SorterLeft(this);
        sorterRight = new SorterRight(this);
        
        colorSorter = new ColorSorter(this);

        time = new ElapsedTime();
        driveController = new DriveController(hardwareMap);
        fireControl = new FireControl(new AprilTagLocalizer(hardwareMap), telemetry);

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
            if (gamepad2.dpad_up && flag4 < time.seconds())
            {
                obeliskTag = 21;
                flag4 = time.seconds() + 0.25;
            }
            else if (gamepad2.dpad_down  && flag4 < time.seconds())
            {
                obeliskTag = 23;
                flag4 = time.seconds() + 0.25;
            }

            if (gamepad2.y && flag5 < time.seconds())
            {
                numberOfBallsScored++;
                flag5 = time.seconds() + 0.25;
            }
            else if (gamepad2.a  && flag5 < time.seconds())
            {
                numberOfBallsScored--;
                flag5 = time.seconds() + 0.25;
            }


            driveController.gamepadController(gamepad1);

            intake.simpleDrive(1, gamepad1.right_trigger > 0.25);

            if (gamepad1.y && flag3 < time.seconds())
            {
                sort = !sort;
                flag3 = time.seconds() + 0.25;
            }

            if (sort)
            {
                if (colorSorter.getColor() == colorSorter.getNextColor(numberOfBallsScored,obeliskTag))
                {
                    
                }
            }
            

            if (gamepad1.right_bumper)
            {
                trigger.driveServo(0.7);
                flag1 = time.seconds() + 0.25;
            }
            else if (flag1 < time.startTime())
            {
                trigger.driveServo(1);
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

            fireControl.firingSuite(12);
            telemetry.addData("Target obelisk tag", obeliskTag);
            shooter.currentDraw();
            telemetry.update();
        }

    }
}
