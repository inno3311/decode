package org.firstinspires.ftc.teamcode.OpModes.Autonomous;//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.vision.VisionPortal;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

//@Autonomous(name="read_obelisk", group="Linear OpMode")
public class read_obelisk extends LinearOpMode
{
    V3ActionsBackpack actionsBackpack;
    AprilTagLocalizer aprilTagLocalizer;

    artifact_rail_detection railDetection;
    List<AprilTagDetection> list;

    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean isBlue = false;

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, true);
        actionsBackpack = new V3ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Trigger(this),
            new Hood(this), new Turret(hardwareMap, telemetry,isBlue), new FireControl(aprilTagLocalizer, telemetry), new ElapsedTime(), new SorterLeft(this), new SorterRight(this),
        new Intake_sort(this), new ColorSensor(hardwareMap),isBlue);

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            boolean found = false;
            int searchCount = 0;
            while (!found && searchCount < 1000)
            {
                list = aprilTagLocalizer.getCurrentDetections();
                telemetry.addData("data", list.toString());
                telemetry.addData("data", searchCount);
                telemetry.update();
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

            telemetry.addData("data", list.toString());
            telemetry.addData("found", found);


            railDetection = new artifact_rail_detection(telemetry);
            double numBalls = railDetection.getNumBalls();
            telemetry.addData("numBalls", numBalls);
            telemetry.update();

            waitForStart();

            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)
                    .afterTime(0, actionsBackpack.readRamp())
                    .waitSeconds(10)
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

