package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


//import org.firstinspires.ftc.teamcode.initialization.Initialization;

import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.CsvLogger;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v1.Transfer;
import org.firstinspires.ftc.teamcode.Roadrunner.ActionsBackpack;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;

@Autonomous(name="RedBack3_C_B", group="Linear OpMode")
public class ZoeRedRun1 extends LinearOpMode
{

    //Initialization initialization;

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

                .afterTime(0, actionsBackpack.mezRampUp(1))
                .afterTime(0, actionsBackpack.mezAction(13, 3, 950, 45)) //shooting 1st time
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155)) //shooting 1st time from back triangle
                .waitSeconds(6.5)
                .afterTime(1,actionsBackpack.intakeBall(1))
                .splineTo(new Vector2d(36,50),Math.toRadians(90),new TranslationalVelConstraint(20)) //1st set pickup
                .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90), new TranslationalVelConstraint(40)) //move to shoot location
                //.afterTime(0, actionsBackpack.mezRampUp(.7))
                .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(130), new TranslationalVelConstraint(40))
                .afterTime(0, actionsBackpack.mezAction(12, 3, 870, 45)) //shooting 2nd time
                .waitSeconds(6)
                .strafeToLinearHeading(new Vector2d(-12, 30), Math.toRadians(80))
                .afterTime(.1,actionsBackpack.intakeBall(1))
                .strafeTo(new Vector2d(-12, 50), new TranslationalVelConstraint(30))
                .afterTime(0, actionsBackpack.mezAction(12, 3, 870, 45)) //shooting 3rd time
                .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(130), new TranslationalVelConstraint(40))

                .waitSeconds(6)
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

