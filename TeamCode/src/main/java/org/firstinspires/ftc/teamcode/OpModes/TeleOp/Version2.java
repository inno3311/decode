package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.Hood;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.Trigger;
import org.firstinspires.ftc.teamcode.Robot.Shooter;
import org.firstinspires.ftc.teamcode.Robot.ColorSorter;
import org.firstinspires.ftc.teamcode.Robot.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.SorterRight;

@TeleOp(name = "Version2")
public class Version2 extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Trigger trigger;
    SorterLeft sorterLeft;
    SorterRight sorterRight;
//    ColorSorter colorSorter;
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
    double shoot_velocity;
    double trigger_state;
    double obeliskTag = 22;
    int numberOfBallsScored = 0;
//    ColorSorter.color[] ballsStored = {ColorSorter.color.none, ColorSorter.color.none, ColorSorter.color.none};

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        trigger = new Trigger(this);
        sorterLeft = new SorterLeft(this);
        sorterRight = new SorterRight(this);
        
//        colorSorter = new ColorSorter(this);

        time = new ElapsedTime();
        driveController = new DriveController(hardwareMap);
        fireControl = new FireControl(new AprilTagLocalizer(hardwareMap), telemetry);

        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
//            if (gamepad2.dpad_up && flag4 < time.seconds())
//            {
//                obeliskTag = 21;
//                flag4 = time.seconds() + 0.25;
//            }
//            else if (gamepad2.dpad_down  && flag4 < time.seconds())
//            {
//                obeliskTag = 23;
//                flag4 = time.seconds() + 0.25;
//            }
            // Drive Code
            driveController.gamepadController(gamepad1);
            // Intake
            intake.simpleDrive(1, gamepad1.right_trigger > 0.25);

            // Color Sorter
            // Manual input on ramp
//            if (gamepad2.y && flag5 < time.seconds())
//            {
//                numberOfBallsScored++;
//                flag5 = time.seconds() + 0.25;
//            }
//            else if (gamepad2.a  && flag5 < time.seconds())
//            {
//                numberOfBallsScored--;
//                flag5 = time.seconds() + 0.25;
//            }

//            // Sorter Toggle

//
//            // Sorter Block
//            if (sort)
//            {
//                trigger.driveServo(0.2);
//                if (colorSorter.getColor() != colorSorter.getNextColor(numberOfBallsScored,obeliskTag))
//                {
//                    if (ballsStored[0] == ColorSorter.color.none)
//                    {
//                        sorterLeft.driveServo(1);
//                    }
//                    else if (ballsStored[2] == ColorSorter.color.none)
//                    {
//                        sorterRight.driveServo(1);
//                    }
//                }
//            }
//
//            // Unloading the sorter
//            if (gamepad2.right_trigger > 0.25)
//            {
//                if (ballsStored[0] == colorSorter.getNextColor(numberOfBallsScored,obeliskTag))
//                {
//                    sorterLeft.driveServo(0);
//                }
//                else if (ballsStored[2] == colorSorter.getNextColor(numberOfBallsScored,obeliskTag))
//                {
//                    sorterLeft.driveServo(1);
//                }
//            }

            if ((gamepad2.a && !gamepad2.start) && flag4 < time.seconds())
            {
                sort = !sort;
                flag4 = time.seconds() + 0.25;
            }


            if (gamepad2.y && flag3 < time.seconds())
            {
                flag3 = time.seconds() + 0.25;
                sorterRight.driveServo(1 - sorterRight.getPosition());
            }
            else if (gamepad2.x && flag3 < time.seconds())
            {
                flag3 = time.seconds() + 0.25;
                sorterLeft.driveServo(1 - sorterLeft.getPosition());
            }


            // The trigger codez

            if (gamepad1.right_bumper)
            {
                if (gamepad1.left_trigger <= 0.1)
                {
                    trigger.driveServo(0.5);
                }
                else if (gamepad1.left_trigger >= 0.8)
                {
                    trigger.driveServo(0.7);
                }
                else
                {
                    trigger.driveServo(0);
                }
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

            //Hood and flywheel adjustment
            shooterParameters = fireControl.firingSuite(initialTargetVelocity);
            shoot_velocity = shooterParameters[1];
            hood.driveToAngleTarget(shooterParameters[0]);
            if (gamepad1.b)
            {
//                shooterParameters = fireControl.firingSuite(initialTargetVelocity);
//                hood.driveToAngleTarget(shooterParameters[0]);
                shooter.driveToVelocity(shoot_velocity);
            }
            else if (gamepad1.a)
            {
                shooter.setPower(0);
            }

            //Telemetry
            telemetry.addData("lift servo_position", trigger.getPosition());
            telemetry.addData("Hood target position", shooterParameters[0]/84.375);
            telemetry.addData("Hood position", hood.getPosition());
            telemetry.addData("actual_vel", shooter.getVelocity());
            telemetry.addData("target_vel", shoot_velocity);
            telemetry.addData("Shoot?", (shoot_velocity - shooter.getVelocity()));
//            telemetry.addData("Target obelisk tag", obeliskTag);
            telemetry.addData("trigger target", Math.pow((1.7*gamepad1.left_trigger)-0.7, 2));
            shooter.currentDraw();
            telemetry.update();
        }
    }
}
