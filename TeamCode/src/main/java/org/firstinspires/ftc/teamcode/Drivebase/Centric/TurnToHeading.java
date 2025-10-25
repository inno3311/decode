package org.firstinspires.ftc.teamcode.Drivebase.Centric;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
//import org.firstinspires.ftc.teamcode.FeedbackSystems.IMU.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class TurnToHeading
{
    Telemetry telemetry;
    MecanumDrive driveController;
    IMU imu;

    private double target;

    public TurnToHeading(Telemetry telemetry, MecanumDrive driveController, IMU imu)
    {
        this.telemetry = telemetry;
        this.driveController = driveController;
        this.imu= imu;
    }

    public double turnToHeading(double x, double y, double deadzone_x, double deadzone_y)
    {
        if (Math.abs(x) <= deadzone_x && Math.abs(y) <= deadzone_y)
        {
            return(0);
        }
        double imu_heading = imu.getRobotYawPitchRollAngles().getYaw();
        double current_heading = Math.signum(imu_heading) * Math.abs(imu_heading)%360;
        double target_heading = Math.toDegrees(Math.atan2(-x, -y));
        if (Math.signum(target_heading) == -1.0)
        {
            target_heading = target_heading + 360;
        }
        if (Math.signum(current_heading) == -1.0)
        {
            current_heading = current_heading + 360;
        }
        double delta_heading = current_heading + 360 - target_heading;
        if (Math.abs(current_heading - target_heading) <= Math.abs(current_heading + 360 - target_heading))
        {
            delta_heading = current_heading - target_heading;
            if (Math.abs(current_heading - target_heading) >= Math.abs(current_heading - 360 - target_heading))
            {
                delta_heading = current_heading - 360 - target_heading;
            }
        }
        delta_heading = delta_heading/90;

        return (delta_heading);
    }

}
