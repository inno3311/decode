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

@Autonomous(name="V3_BlueFront3_A", group="Linear OpMode")
public class V3BlueFront3_A extends LinearOpMode
{
    public static final String ALLIANCE = "Alliance";
    public static final String ENDPOSE = "Pose";

    V3ActionsBackpack actionsBackpack;

    AprilTagLocalizer aprilTagLocalizer;

    List<AprilTagDetection> list;

    @Override
    public void runOpMode() throws InterruptedException
    {

        boolean isBlue = true;
        blackboard.put(ALLIANCE, "BLUE");

        //OpenCvCamera camera;
        //artifact_rail_detection pipeline;
        Turret turret = new Turret(hardwareMap, telemetry,isBlue);


        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, true);
        actionsBackpack = new V3ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Flipper(this),
            new Hood(this), turret, new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime(), new SorterLeft(this), new SorterRight(this),
        new Intake_sort(this), new ColorSensor(hardwareMap),isBlue);

        turret.trimX(4);

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(-42, -51, Math.toRadians(90+33));

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
                telemetry.addLine("!!! Team is BLUE !!!");
                telemetry.update();
                idle();

            }


            waitForStart();

            try
            {




            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)
                .afterTime(0, actionsBackpack.shootBallManual(9,3, 1050,35, drive))
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(270)) //launch zone
                .waitSeconds(8)
                .afterTime(0,actionsBackpack.intakeColor(7))
                .strafeToLinearHeading(new Vector2d(-10, -55), Math.toRadians(270), new TranslationalVelConstraint(25)) //A
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(270)) //launch zone
                .afterTime(0, actionsBackpack.shootBallManual(9,3, 1050,35, drive))
                .waitSeconds(8)
                .strafeToLinearHeading(new Vector2d(10, -10), Math.toRadians(270)) //in line with B

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

