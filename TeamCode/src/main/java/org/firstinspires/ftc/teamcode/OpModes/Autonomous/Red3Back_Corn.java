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
import org.firstinspires.ftc.teamcode.FeedbackSystems.ColorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Roadrunner.V3ActionsBackpack;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;

@Autonomous(name="RedBack3_Corn", group="Linear OpMode")
public class Red3Back_Corn extends LinearOpMode
{
    public static final String ALLIANCE = "Alliance";
    public static final String ENDPOSE = "Pose";

    AutoConsts cons;

    V3ActionsBackpack actionsBackpack;


    @Override
    public void runOpMode() throws InterruptedException
    {
        blackboard.put(ALLIANCE, "RED");

        actionsBackpack = new V3ActionsBackpack(new Shooter(hardwareMap, telemetry), new Intake(this), new Trigger(this),
            new Hood(this), new Turret(hardwareMap, telemetry), new FireControl(new AprilTagLocalizer(hardwareMap), telemetry),
                new ElapsedTime(), new SorterLeft(this), new SorterRight(this), new Intake_sort(this), new ColorSensor(hardwareMap));

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {

            waitForStart();

            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)

                .afterTime(0, actionsBackpack.shootBall(9,3, drive.localizer.getPose(), false))
//                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155))
                .waitSeconds(10)

                //picking up from corner
                .afterTime(1,actionsBackpack.intakeBall(-1))
                .turnTo(Math.toRadians (90))
                .afterTime(0.1,actionsBackpack.intakeBall(-1))
                .strafeTo(new Vector2d(55, 70)) //hit corner balls.
                .afterTime(0.1,actionsBackpack.intakeBall(-1))
                .strafeTo(new Vector2d(55, 50)) //back up
                .afterTime(0.1,actionsBackpack.intakeBall(-1))
                .strafeTo(new Vector2d(60, 70)) //hit again
                .afterTime(0, actionsBackpack.shootBall(10,3, drive.localizer.getPose(), false))
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

        blackboard.put(ENDPOSE, drive.localizer.getPose());

    }
}

