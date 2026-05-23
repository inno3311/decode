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

@Autonomous(name="V4_BlueFront_A", group="Linear OpMode")
public class V4BlueFront_A extends LinearOpMode
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

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        actionsBackpack = new V4ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Trigger(hardwareMap),
            new Hood(this), new Turret(hardwareMap, telemetry,isBlue), new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime(),isBlue);

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(-50, -50, Math.toRadians(225));

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

                    .afterTime(0, actionsBackpack.turretTracking(drive,3))
                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
                    .strafeToLinearHeading(new Vector2d(-11, -12), Math.toRadians(270), new TranslationalVelConstraint(40))
                    .waitSeconds(1.5)
                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    .strafeToLinearHeading(new Vector2d(-11, -56), Math.toRadians(270), new TranslationalVelConstraint(40)) //A
                    .strafeToLinearHeading(new Vector2d(-11, -12), Math.toRadians(270), new TranslationalVelConstraint(40))
                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
                    .waitSeconds(2)

                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(12, -20), Math.toRadians(270), new TranslationalVelConstraint(20))
                    .splineToConstantHeading(new Vector2d(12, -66), Math.toRadians(270), new TranslationalVelConstraint(40))

                    //.waitSeconds(1)
                    .splineToConstantHeading(new Vector2d(12, -40), Math.toRadians(90), new TranslationalVelConstraint(40))
                    .splineToConstantHeading(new Vector2d(-10, -12), Math.toRadians(190), new TranslationalVelConstraint(30))
                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
                    .waitSeconds(2)

                    .afterTime(0, actionsBackpack.intakeBall(-1))
                    .strafeToLinearHeading(new Vector2d(32, -64), Math.toRadians(180), new TranslationalVelConstraint(40))
                    .waitSeconds(2)
                    .strafeToLinearHeading(new Vector2d(28, -64), Math.toRadians(180), new TranslationalVelConstraint(40))
                    .strafeToLinearHeading(new Vector2d(-8, -12), Math.toRadians(270), new TranslationalVelConstraint(30))
                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
                    .waitSeconds(2)
                    .strafeToLinearHeading(new Vector2d(10, -10), Math.toRadians(270))

//                    .strafeToLinearHeading(new Vector2d(-10, -12), Math.toRadians(270), new TranslationalVelConstraint(40))
//                    .afterTime(0, actionsBackpack.turretTracking(drive,-3))
//                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
//                    .waitSeconds(6)
//                    .afterTime(0, actionsBackpack.intakeBall(-1))
//                    .strafeToLinearHeading(new Vector2d(-10, -56), Math.toRadians(270), new TranslationalVelConstraint(40)) //A
//                    .strafeToLinearHeading(new Vector2d(-10, -12), Math.toRadians(270), new TranslationalVelConstraint(40))
//                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
//                    .waitSeconds(3)
//
//                    .strafeToLinearHeading(new Vector2d(12, -10), Math.toRadians(270))
//                    .afterTime(0, actionsBackpack.intakeBall(-1))
//                    .strafeTo(new Vector2d(12, -70))
//                    //.waitSeconds(1)
//                    .strafeToLinearHeading(new Vector2d(12, -40), Math.toRadians(270), new TranslationalVelConstraint(40))
//                     .strafeToLinearHeading(new Vector2d(-10, -12), Math.toRadians(270), new TranslationalVelConstraint(40))
//                    .afterTime(0, actionsBackpack.shootBallManual(9, -3, 1100, 20, drive))
//                    .waitSeconds(3)
//                    .strafeToLinearHeading(new Vector2d(10, -10), Math.toRadians(270))



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

