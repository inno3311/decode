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


//import org.firstinspires.ftc.teamcode.initialization.Initialization;

import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Lift;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;
import org.firstinspires.ftc.teamcode.Roadrunner.ActionsBackpack;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;

@Autonomous(name="Autonomous 4 balls in goal", group="Linear OpMode")
public class ZoeRedRun1 extends LinearOpMode
{

    //Initialization initialization;

    ActionsBackpack actionsBackpack;


    @Override
    public void runOpMode() throws InterruptedException
    {
        actionsBackpack = new ActionsBackpack(new Shooter(this), new Intake(this), new Lift(this),
            new Hood(this), new Transfer(this), new FireControl(new AprilTagLocalizer(hardwareMap), telemetry), new ElapsedTime());

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)

                // Zoe, your path starts here!   GO FORTH AND CODE!!!!!!!
                //.strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(225), new TranslationalVelConstraint(30))
                //.turnTo(Math.toRadians(135))
                //.afterTime(0, actionsBackpack.fireBall(10))
                //.afterTime(0,actionsBackpack.intakeBall(1))
                //.splineTo(new Vector2d(0,0),Math.toRadians(180),new TranslationalVelConstraint(20))
                //.waitSeconds(1)
                //.afterTime(0, actionsBackpack.intakeBall(0))
                .turnTo(Math.toRadians(155))
                .afterTime(0, actionsBackpack.fireBall(12, 2))
                .waitSeconds(10)
                .afterTime(0, actionsBackpack.transferBall(1))
                .afterTime(0,actionsBackpack.intakeBall(1))
                .splineTo(new Vector2d(36,51),Math.toRadians(90),new TranslationalVelConstraint(20))
                .afterTime(0, actionsBackpack.transferBall(0))
                .afterTime(0,actionsBackpack.intakeBall(0))
                .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(-12, 15), Math.toRadians(135), new TranslationalVelConstraint(30))
                .afterTime(0, actionsBackpack.fireBall(8, 2))
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

