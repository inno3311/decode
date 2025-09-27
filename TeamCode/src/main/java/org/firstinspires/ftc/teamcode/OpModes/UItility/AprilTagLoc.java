package org.firstinspires.ftc.teamcode.OpModes.UItility;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;

@Autonomous(name = "AprilTagLoc")
@Disabled
public class AprilTagLoc extends LinearOpMode
{
    AprilTagLocalizer aprilTagLocalizer;

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            aprilTagLocalizer.tagsTelemetry(telemetry);
        }

    }
}
