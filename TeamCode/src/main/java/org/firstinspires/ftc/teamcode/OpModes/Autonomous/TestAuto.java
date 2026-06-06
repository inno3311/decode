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

@Autonomous(name="TestAuto", group="Linear OpMode")
public class TestAuto extends LinearOpMode
{
    public static final String ALLIANCE = "Alliance";
    public static final String ENDPOSE = "Pose";

    V4ActionsBackpack actionsBackpack;

    AprilTagLocalizer aprilTagLocalizer;

    List<AprilTagDetection> list;

    @Override
    public void runOpMode() throws InterruptedException
    {

        blackboard.put(ALLIANCE, "BLUE");
        boolean isBlue = true;

        //OpenCvCamera camera;
        //artifact_rail_detection pipeline;
        Turret turret = new Turret(hardwareMap, telemetry,isBlue);
        turret.trimX(8);
        //turret.trimY(-6);

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        actionsBackpack = new V4ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Trigger(hardwareMap),
            new Hood(this), turret, new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime(),isBlue);

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, -15, Math.toRadians(180));

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
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.turretTracking(drive,0)) //-8
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-5))
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-10))
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-15))
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-20))
                    .waitSeconds(1)
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-15))
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-10))
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-5))
                    .waitSeconds(1)
                    .afterTime(0, actionsBackpack.adjustTurretOffset(-0))
                    .waitSeconds(1)



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

