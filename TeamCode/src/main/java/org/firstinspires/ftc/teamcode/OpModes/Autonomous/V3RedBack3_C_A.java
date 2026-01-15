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


//import org.firstinspires.ftc.teamcode.initialization.Initialization;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_rail_detection;
import org.firstinspires.ftc.teamcode.FeedbackSystems.ColorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Roadrunner.V3ActionsBackpack;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name="V3_RedBack3_C_A", group="Linear OpMode")
public class V3RedBack3_C_A extends LinearOpMode
{
    public static final String ALLIANCE = "Alliance";
    public static final String ENDPOSE = "Pose";

    V3ActionsBackpack actionsBackpack;

    AprilTagLocalizer aprilTagLocalizer;

    List<AprilTagDetection> list;

    @Override
    public void runOpMode() throws InterruptedException
    {

        blackboard.put(ALLIANCE, "RED");
        boolean isBlue = false;

        //OpenCvCamera camera;
        //artifact_rail_detection pipeline;

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, true);
        actionsBackpack = new V3ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Trigger(this),
            new Hood(this), new Turret(hardwareMap, telemetry,isBlue), new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime(), new SorterLeft(this), new SorterRight(this),
        new Intake_sort(this), new ColorSensor(hardwareMap),isBlue);

        // ZOE update with starting location
        Pose2d beginPose = new Pose2d(60, 15, Math.toRadians(180));

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


            while (!found && searchCount < 5000)
            {
//                list = aprilTagLocalizer.getCurrentDetections();
//                if (!list.isEmpty())
//                {
//                    AprilTagDetection targetTag = null;
//
//                    for (AprilTagDetection detection : list) {
//                        int id = detection.id;
//
//                        if (id >= 21 && id <= 23) {
//                            targetTag = detection;
//                            found = true;
//                            telemetry.addData("id", id);
//                            telemetry.update();
//                            actionsBackpack.taglist = list;
//                            actionsBackpack.aprilTag_Id = id;
//                            break; // stop once found
//                        }
//                    }
//                }
//                else {
//                    searchCount++;
//                    telemetry.addData("searchCount", searchCount);
//                    telemetry.update();
//                }
            }

            // -------------------------------
            // 1. Camera setup
            // -------------------------------
//            int cameraMonitorViewId = hardwareMap.appContext
//                .getResources()
//                .getIdentifier(
//                    "cameraMonitorViewId",
//                    "id",
//                    hardwareMap.appContext.getPackageName()
//                );
//
//            camera = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"),
//                cameraMonitorViewId
//            );
//
//            // -------------------------------
//            // 2. Create & attach pipeline
//            // -------------------------------
//            pipeline = new artifact_rail_detection(telemetry);
//            camera.setPipeline(pipeline);
//
//            // -------------------------------
//            // 3. Open camera async
//            // -------------------------------
//            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//                    telemetry.addData("Camera error", errorCode);
//                }
//            });

            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)
                //.afterTime(0, actionsBackpack.read_obelisk(list))  // read obelist.  This may no longer be needed
                .afterTime(0, actionsBackpack.shootBallManual(9,3, 1400,35, drive))  //fire first set of three balls
//                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(180))
                .waitSeconds(7)
                .afterTime(0,actionsBackpack.intakeColor(7))
                .splineTo(new Vector2d(36,70),Math.toRadians(90),new TranslationalVelConstraint(15)) //1st set pickup
                .waitSeconds(.5)
//                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90), new TranslationalVelConstraint(40)) //move to shoot location
//                .splineTo(new Vector2d(36,30),Math.toRadians(90),new TranslationalVelConstraint(40))
                .afterTime(0, actionsBackpack.shootBallManual(9,3, 1050,35, drive))  //shooting 2nd time
                .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(90), new TranslationalVelConstraint(40))
//                .splineTo(new Vector2d(-12,20),Math.toRadians(90),new TranslationalVelConstraint(40))
                .waitSeconds(7)
                .afterTime(0,actionsBackpack.intakeColor(7))
                .strafeTo(new Vector2d(-12, 55))
                //.strafeTo(new Vector2d(-12, 20))
//                .afterTime(0, actionsBackpack.shootBallManual(9,3, 1150,55, drive))
                //.waitSeconds(7)
//                .strafeToLinearHeading(new Vector2d(-12, 30), Math.toRadians(80))
//                .strafeTo(new Vector2d(-12, 65), new TranslationalVelConstraint(20))
//                .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(125), new TranslationalVelConstraint(40))
                .waitSeconds(4)
//                .strafeToLinearHeading(new Vector2d(0, 45), Math.toRadians(180))
                ; //do not remove ;


            Action redRun = yellow_drop
                .build();

            Actions.runBlocking(redRun);

            blackboard.put(ENDPOSE, drive.localizer.getPose());

        }
        else
        {
            throw new RuntimeException();
        }


    }
}

