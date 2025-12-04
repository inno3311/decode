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

@Autonomous(name="AutoTest", group="Linear OpMode")
public class TestAuto extends LinearOpMode
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


//                .afterTime(0, actionsBackpack.setHood(30))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.setHood(0))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.setHood(30))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.setHood(0))
//                .waitSeconds(3)

                //.turnTo(Math.toRadians(155))
                //.afterTime(0, actionsBackpack.mezAction(12, 2))
                .afterTime(0, actionsBackpack.mezAction(13,3,800,50))
                .waitSeconds(15)


//                .afterTime(0, actionsBackpack.trigger(1))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.trigger(0))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.trigger(1))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.trigger(0))
//                .waitSeconds(3)



//                .afterTime(0, actionsBackpack.target(12))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.target(12))
//                .waitSeconds(3)
//                .afterTime(0, actionsBackpack.target(12))
//                .waitSeconds(3)



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

