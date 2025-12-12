package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

@Autonomous(name="RedBack3_C", group="Linear OpMode")
public class RedBack3_C extends LinearOpMode
{
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
                //.afterTime(0,actionsBackpack.mezRampUp(1))
                .afterTime(0, actionsBackpack.mezAction(13, 3, 950, 45)) //shooting 1st time
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155)) //shooting 1st time from back triangle
                .waitSeconds(7)
                .afterTime(0,actionsBackpack.intakeBall(1))
                .afterTime(1,actionsBackpack.intakeBall(1))
                .afterTime(2,actionsBackpack.intakeBall(1))
                .splineTo(new Vector2d(36,-60),Math.toRadians(90),new TranslationalVelConstraint(20)) //1st set pickup
                .afterTime(.5, actionsBackpack.mezAction(12, 3, 950, 45)) //shooting 2nd time
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155), new TranslationalVelConstraint(40))
                .waitSeconds(7)
                .afterTime(0,actionsBackpack.intakeBall(1))
                .afterTime(1,actionsBackpack.intakeBall(1))
                .strafeToLinearHeading(new Vector2d(12, 30), Math.toRadians(90))
                .afterTime(0,actionsBackpack.intakeBall(1))
                .strafeTo(new Vector2d(12, 60), new TranslationalVelConstraint(20))
                .waitSeconds(2)
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

