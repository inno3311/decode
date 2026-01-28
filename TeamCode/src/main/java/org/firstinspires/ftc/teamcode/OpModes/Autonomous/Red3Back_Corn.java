package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_rail_detection;
import org.firstinspires.ftc.teamcode.FeedbackSystems.ColorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Roadrunner.V3ActionsBackpack;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Flipper;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

//@Autonomous(name="RedBack3_Corn", group="Linear OpMode")
public class Red3Back_Corn extends LinearOpMode
{
    public static final String ALLIANCE = "Alliance";
    public static final String ENDPOSE = "Pose";

    AutoConsts cons;

    V3ActionsBackpack actionsBackpack;

    AprilTagLocalizer aprilTagLocalizer;

    List<AprilTagDetection> list;

    artifact_rail_detection pipeline;


    @Override
    public void runOpMode() throws InterruptedException
    {
        blackboard.put(ALLIANCE, "RED");

        boolean isBlue = false;

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, true);
        actionsBackpack = new V3ActionsBackpack(new Shooter(hardwareMap, telemetry), new Intake(this), new Flipper(this),
            new Hood(this), new Turret(hardwareMap, telemetry,isBlue), new FireControl(aprilTagLocalizer, telemetry),
                new ElapsedTime(), new SorterLeft(this), new SorterRight(this), new Intake_sort(this), new ColorSensor(hardwareMap),isBlue);

        // ZOE update with starting location
        //Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {

            waitForStart();

            boolean found = false;
            int searchCount = 0;
            while (!found && searchCount < 100)
            {
                list = aprilTagLocalizer.getCurrentDetections();
                if (!list.isEmpty())
                {
                    AprilTagDetection targetTag = null;

                    for (AprilTagDetection detection : list) {
                        int id = detection.id;

                        if (id >= 21 && id <= 23) {
                            targetTag = detection;
                            found = true;
                            break; // stop once found
                        }
                    }
                }
                else {
                    searchCount++;
                }
            }

            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)
                .afterTime(0, actionsBackpack.read_obelisk(list))
//                .afterTime(0, actionsBackpack.shootBall(9,3, drive.localizer.getPose(), false, drive))
            //    .afterTime(0, actionsBackpack.shootBallManual(9,3, 1500,35, drive))  //good for back zone
                .afterTime(0, actionsBackpack.shootBallManual(9,3, 1150,55, drive))  //good for back zone
//                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155))
                .waitSeconds(10)

//                .afterTime(1,actionsBackpack.intakeBall(-1))
//                .splineTo(new Vector2d(36,52),Math.toRadians(90),new TranslationalVelConstraint(10))
//                .afterTime(1,actionsBackpack.intakeBall(0))


                //picking up from corner
//                .afterTime(1,actionsBackpack.intakeBall(-1))
//                .turnTo(Math.toRadians (90))
//                .afterTime(0.1,actionsBackpack.intakeBall(-1))
//                .strafeTo(new Vector2d(55, 70)) //hit corner balls.
//                .afterTime(0.1,actionsBackpack.intakeBall(-1))
//                .strafeTo(new Vector2d(55, 50)) //back up
//                .afterTime(0.1,actionsBackpack.intakeBall(-1))
//                .strafeTo(new Vector2d(60, 70)) //hit again
//                .afterTime(0, actionsBackpack.shootBall(10,3, drive.localizer.getPose(), false, drive))
//                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(155))
//                .waitSeconds(5)
//                .strafeToLinearHeading(new Vector2d(50, 40 ), Math.toRadians(180))

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

