package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Roadrunner.ActionsBackpack;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v1.Transfer;

@Autonomous(name="RedBack3_Corn", group="Linear OpMode")
public class Red3Back_Corn extends LinearOpMode
{

    AutoConsts cons;

    ActionsBackpack actionsBackpack;


    @Override
    public void runOpMode() throws InterruptedException
    {
        actionsBackpack = new ActionsBackpack(new Shooter(this), new Intake(this), new Trigger(this),
            new Hood(this), new Transfer(this), new FireControl(new AprilTagLocalizer(hardwareMap), telemetry), new ElapsedTime());

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)

                .afterTime(0, actionsBackpack.mezAction(13, 3, 950, 45))
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155))
                .waitSeconds(7)

                //picking up from corner
                .afterTime(1,actionsBackpack.intakeBall(1))
                .turnTo(Math.toRadians (90))
                .afterTime(0.1,actionsBackpack.intakeBall(1))
                .strafeTo(new Vector2d(55, 70)) //hit corner balls.
                .afterTime(0.1,actionsBackpack.intakeBall(1))
                .strafeTo(new Vector2d(55, 50)) //back up
                .afterTime(0.1,actionsBackpack.intakeBall(1))
                .strafeTo(new Vector2d(60, 70)) //hit again
                .afterTime(0, actionsBackpack.mezAction(13, 2, 950, 45))//
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155))
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(50, 40 ), Math.toRadians(180))

                ; //do not remove ;



            Action redRun = yellow_drop
                .build();

            Actions.runBlocking(redRun);

        }
        else
        {
            throw new RuntimeException();
        }
    }
}

