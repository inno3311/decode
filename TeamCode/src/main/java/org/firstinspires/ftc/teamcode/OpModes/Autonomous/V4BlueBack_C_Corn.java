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

@Autonomous(name="V4_BlueBack_C_Corn", group="Linear OpMode")
public class V4BlueBack_C_Corn extends LinearOpMode
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
                telemetry.addLine("!!! Team is RED !!!");
                telemetry.update();
                idle();



            }

            waitForStart();

            try
            {


                TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)
                    .afterTime(0, actionsBackpack.shootBallManual(9, 3, 1500, 30, drive)) //at launch zone
                    .splineTo(new Vector2d(50, -15), Math.toRadians(180), new TranslationalVelConstraint(10))
                    //.afterTime(0, actionsBackpack.turretTracking(drive))
                    .waitSeconds(6)
                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    .splineTo(new Vector2d(36, -60), Math.toRadians(270), new TranslationalVelConstraint(40)) // C
                    .waitSeconds(.5)
                    //first set, fires 3, intakes 3.
                    .strafeToLinearHeading(new Vector2d(50, -15), Math.toRadians(270), new TranslationalVelConstraint(40)) //launch zone
                    .afterTime(0, actionsBackpack.shootBallManual(9, 3, 1500, 35, drive))
                    .waitSeconds(3)

                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    //.turnTo(90)
                    .strafeToLinearHeading(new Vector2d(65, -75), Math.toRadians(260), new TranslationalVelConstraint(40)) //corner
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(50, -15), Math.toRadians(270), new TranslationalVelConstraint(40))
                    .afterTime(0, actionsBackpack.shootBallManual(9, 3, 1500, 35, drive))
                    .waitSeconds(5)
                    .strafeToLinearHeading(new Vector2d(30, -15), Math.toRadians(270), new TranslationalVelConstraint(40))







//                    .afterTime(0, actionsBackpack.intakeBall(1))
//                    .strafeTo(new Vector2d(-10, 50), new TranslationalVelConstraint(20)) // A
                    //second set, fires 3, intakes 3,
       //             .afterTime(0, actionsBackpack.shootBallManual(9, 3, 1050, 35, drive))
       //             .strafeTo(new Vector2d(-10, 20)) //launch zone
       //             .waitSeconds(8)
       //             .strafeToLinearHeading(new Vector2d(10, 10), Math.toRadians(86)) //in line with B
                    //third set, fires 3, moves.


//                .waitSeconds(4)
//                .strafeToLinearHeading(new Vector2d(0, 45), Math.toRadians(180))
//                .splineTo(new Vector2d(-12,20),Math.toRadians(90),new TranslationalVelConstraint(40))

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

