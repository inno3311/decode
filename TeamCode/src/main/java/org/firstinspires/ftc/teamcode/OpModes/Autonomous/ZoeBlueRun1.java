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

@Autonomous(name="Zoe Blue1", group="Linear OpMode")
public class ZoeBlueRun1 extends LinearOpMode
{

    //Initialization initialization;

    ActionsBackpack actionsBackpack;


    @Override
    public void runOpMode() throws InterruptedException
    {
        actionsBackpack = new ActionsBackpack(new Shooter(this), new Intake(this), new Trigger(this),
            new Hood(this), new Transfer(this), new FireControl(new AprilTagLocalizer(hardwareMap), telemetry), new ElapsedTime());

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, -15, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)

                //.afterTime(0, actionsBackpack.mezRampUp(1))

                .afterTime(0, actionsBackpack.mezAction(13, 2, 1400, 35))
                .strafeToLinearHeading(new Vector2d(50, -15), Math.toRadians(205))
                //.turnTo(Math.toRadians(155))
                .waitSeconds(1)

                .waitSeconds(6.5)

                //.afterTime(0, actionsBackpack.transferBall(1))

                .afterTime(1,actionsBackpack.intakeBall(1))
                .splineTo(new Vector2d(36,-45),Math.toRadians(270),new TranslationalVelConstraint(20))
                .strafeToLinearHeading(new Vector2d(36, -30), Math.toRadians(270), new TranslationalVelConstraint(40))
                .afterTime(0, actionsBackpack.mezRampUp(.7))
                .strafeToLinearHeading(new Vector2d(-12, -20), Math.toRadians(230), new TranslationalVelConstraint(40))
                .afterTime(0, actionsBackpack.mezAction(12, 2, 1150, 8))
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(36, -36), Math.toRadians(180))
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

