package org.firstinspires.ftc.teamcode.OpModes.UItility;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;


import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.opencv.core.Point;

//@Disabled
public class ground_align
{
    MecanumDrive drive;

    public ground_align(MecanumDrive drive)
    {
        this.drive = drive;
    }
    public Action align(Point rel_pos)
    {
        double rel_x = rel_pos.x;
        double rel_y = rel_pos.y;

        TrajectoryActionBuilder align_to_artifact = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .strafeToLinearHeading(new Vector2d(rel_x, rel_y), Math.atan2(rel_x, rel_y))
                .turn(Math.atan2(rel_y, rel_x))
                ;


        return align_to_artifact.build();
    }
}
