package org.firstinspires.ftc.teamcode.OpModes.Autonomous;//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

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
import org.firstinspires.ftc.teamcode.Roadrunner.V4ActionsBackpack;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.teamcode.Robot.v4.Trigger;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="V4_RedFront_Gate ", group="Linear OpMode")
public class V4RedFront_Gate extends LinearOpMode
{
    public static final String ALLIANCE = "Alliance";
    public static final String ENDPOSE = "Pose";

    V4ActionsBackpack actionsBackpack;

    AprilTagLocalizer aprilTagLocalizer;

    List<AprilTagDetection> list;

    @Override
    public void runOpMode() throws InterruptedException
    {

        blackboard.put(ALLIANCE, "RED");
        boolean isBlue = false;

        //OpenCvCamera camera;
        //artifact_rail_detection pipeline;

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        actionsBackpack = new V4ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Trigger(hardwareMap),
            new Hood(this), new Turret(hardwareMap, telemetry,isBlue), new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime(),isBlue);

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(-50, 50, Math.toRadians(135));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            boolean found = false;
            int searchCount = 0;

            while (opModeInInit()) {
                list = aprilTagLocalizer.getCurrentDetections();
                if (!list.isEmpty())
                {
                    AprilTagDetection targetTag = null;

                    for (AprilTagDetection detection : list) {
                        int id = detection.id;

                        if (id >= 21 && id <= 23) {
                            targetTag = detection;
                            found = true;
                            telemetry.addData("id", id);
                            telemetry.update();
                            actionsBackpack.taglist = list;
                            actionsBackpack.aprilTag_Id = id;
                            break; // stop once found
                        }
                    }
                }
                else {
                    searchCount++;
                    telemetry.addData("searchCount", searchCount);
                }
                telemetry.addLine("!!! Team is RED !!!");
                telemetry.update();
                idle();



            }

            waitForStart();

            try
            {


                TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)

                    .afterTime(0, actionsBackpack.turretTracking(drive,4))
                    .strafeToLinearHeading(new Vector2d(-10, 12), Math.toRadians(90), new TranslationalVelConstraint(40))
                    .afterTime(0, actionsBackpack.shootBallManual(9, 3, 1150, 20, drive))
                    .waitSeconds(6)

                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    .strafeToLinearHeading(new Vector2d(12, 10), Math.toRadians(90)) //B
                    .strafeTo(new Vector2d(12, 64))
                    .strafeTo(new Vector2d(12, 45))
                    .strafeToLinearHeading(new Vector2d(4, 57), Math.toRadians(90)) //Gate
                    .strafeToLinearHeading(new Vector2d(-5, 10), Math.toRadians(90), new TranslationalVelConstraint(40))
                    .afterTime(0, actionsBackpack.shootBallManual(9, 3, 1150, 20, drive))
                    .waitSeconds(3)

                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    .strafeToLinearHeading(new Vector2d(-10, 56), Math.toRadians(90), new TranslationalVelConstraint(40)) //A
                    .strafeToLinearHeading(new Vector2d(-10, 12), Math.toRadians(90), new TranslationalVelConstraint(40))
                    .afterTime(0, actionsBackpack.shootBallManual(9, 4, 1150, 20, drive))
                    .waitSeconds(3)

                    .strafeToLinearHeading(new Vector2d(10, 10), Math.toRadians(90))




                    ; //do not remove ;


                Action redRun = yellow_drop
                    .build();

                Actions.runBlocking(redRun);

                blackboard.put(ENDPOSE, drive.localizer.getPose());

            }
            finally
            {
                blackboard.put(ENDPOSE, drive.localizer.getPose());
            }

        }
        else
        {
            throw new RuntimeException();
        }


    }
}

