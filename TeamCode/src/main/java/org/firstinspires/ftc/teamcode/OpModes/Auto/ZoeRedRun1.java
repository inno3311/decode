package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//import org.firstinspires.ftc.teamcode.initialization.Initialization;

import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;

@Autonomous(name="ZoeRun1", group="Linear OpMode")
public final class ZoeRedRun1 extends LinearOpMode
{

    //Initialization initialization;



    @Override
    public void runOpMode() throws InterruptedException
    {
        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)

                // Zoe, your path starts here!   GO FORTH AND CODE!!!!!!!
                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(225), new TranslationalVelConstraint(30))
                .waitSeconds(1)
                .waitSeconds(1)
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

