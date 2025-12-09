package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous(name="Zoe Red Solo", group="Linear OpMode")
public class RedSoloRun extends LinearOpMode
{

    //Initialization initialization;

    ActionsBackpack actionsBackpack;


    @Override
    public void runOpMode() throws InterruptedException
    {

        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        actionsBackpack = new ActionsBackpack(new Shooter(this), new Intake(this), new Trigger(this),
            new Hood(this), new Transfer(this), new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime());

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));

        int targetID = aprilTagLocalizer.getDetectionID();
        telemetry.addData("targetID", targetID);
        telemetry.update();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            int ObeliskPattern = 1; //green, purple, purple.


            TrajectoryActionBuilder starting_action = drive.actionBuilder(beginPose)

                .afterTime(0, actionsBackpack.mezRampUp(1))
                .afterTime(0, actionsBackpack.mezAction(13, 2, 1450, 8))
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155))
                .waitSeconds(6.5);


            TrajectoryActionBuilder pattern1 = drive.actionBuilder(beginPose)

                .afterTime(1,actionsBackpack.intakeBall(1))
                .splineTo(new Vector2d(36,45),Math.toRadians(90),new TranslationalVelConstraint(20))
                .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90), new TranslationalVelConstraint(40))
                .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(130), new TranslationalVelConstraint(40))
                .afterTime(0, actionsBackpack.mezAction(12, 2, 1150, 8))
                .waitSeconds(5);

            TrajectoryActionBuilder pattern2 = drive.actionBuilder(beginPose);;

                pattern2.afterTime(1,actionsBackpack.intakeBall(1));
                pattern2.splineTo(new Vector2d(10,45),Math.toRadians(90),new TranslationalVelConstraint(20));
                pattern2.strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(130), new TranslationalVelConstraint(40));
                pattern2.afterTime(0, actionsBackpack.mezAction(12, 2, 1150, 8));
                pattern2.waitSeconds(5);

            TrajectoryActionBuilder pattern3 = drive.actionBuilder(beginPose);;
                pattern3.afterTime(1,actionsBackpack.intakeBall(1));
                pattern3.splineTo(new Vector2d(-12,45),Math.toRadians(90),new TranslationalVelConstraint(20));
                pattern3.strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(130), new TranslationalVelConstraint(40));
                pattern3.afterTime(0, actionsBackpack.mezAction(12, 2, 1150, 8));
                pattern3.waitSeconds(5);

            Action trajectoryActionChosen;
            if (ObeliskPattern == 1) {
                trajectoryActionChosen = pattern1.build();
            } else if (ObeliskPattern == 2) {
                trajectoryActionChosen = pattern2.build();
            } else {
                trajectoryActionChosen = pattern3.build();
            }



            Action startingRun = starting_action
                .build();

            Actions.runBlocking(
                new SequentialAction(
                    startingRun,
                    trajectoryActionChosen

                ));

        }
        else
        {
            throw new RuntimeException();
        }
    }
}

