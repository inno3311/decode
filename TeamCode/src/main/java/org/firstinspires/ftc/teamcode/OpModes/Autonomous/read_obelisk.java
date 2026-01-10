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
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;

@Autonomous(name="read_obelisk", group="Linear OpMode")
public class read_obelisk extends LinearOpMode
{
    V3ActionsBackpack actionsBackpack;

    @Override
    public void runOpMode() throws InterruptedException
    {
        actionsBackpack = new V3ActionsBackpack(new Shooter(hardwareMap,telemetry), new Intake(this), new Trigger(this),
            new Hood(this), new Turret(hardwareMap, telemetry), new FireControl(new AprilTagLocalizer(hardwareMap), telemetry), new ElapsedTime(), new SorterLeft(this), new SorterRight(this),
        new Intake_sort(this), new ColorSensor(hardwareMap));

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class))
        {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            TrajectoryActionBuilder yellow_drop = drive.actionBuilder(beginPose)
                    .afterTime(0, actionsBackpack.read_obelisk())
                    .waitSeconds(6)
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

