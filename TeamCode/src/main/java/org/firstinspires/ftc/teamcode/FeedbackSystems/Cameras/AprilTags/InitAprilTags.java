package org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumSynchronousDriver;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class InitAprilTags
{
    AprilTagProcessor aprilTagProcessor;
    AprilTagMaster aprilTagMaster;
    ElapsedTime elapsedTime;

    public void initAprilTags(MecanumSynchronousDriver driver, HardwareMap hardwareMap, Telemetry telemetry)
    {
        elapsedTime = new ElapsedTime();

        aprilTagMaster = new AprilTagMaster(hardwareMap);
    }

    public AprilTagMaster getAprilTagMaster()
    {
        return aprilTagMaster;
    }

}
