package org.firstinspires.ftc.teamcode.algorithms;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;


public class autoAim
{
    MecanumDrive drive;

    public autoAim(MecanumDrive drive)
    {
        this.drive = drive;
    }

    public Action aimBot(double heading, int tag)
    {
        Pose2d pose = new Pose2d(0,0, 0);
        TrajectoryActionBuilder trajectory = drive.actionBuilder(pose)
            .turn(heading);

        return trajectory.build();
    }

}
