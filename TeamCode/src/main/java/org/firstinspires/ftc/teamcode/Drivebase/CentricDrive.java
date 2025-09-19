package org.firstinspires.ftc.teamcode.Drivebase;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

public class CentricDrive
{
    Telemetry telemetry;
    MecanumDrive driveController;
    private boolean flag;
    private double lastChanged = 0;

    public CentricDrive(MecanumDrive driveController, Telemetry telemetry)
    {
        this.driveController = driveController;
        this.telemetry = telemetry;
    }

    public void drive(double x,double y, double robot_heading, double slowMo, double turn)
    {
        // double speed = 1 * (1-Range.clip(slowMo, 0, 0.7));
        // double speed = (2*Math.pow(slowMo-1.08*(Math.sqrt(0.3)), 2)) + 0.3;
         double speed = 8*Math.pow((slowMo-0.543879), 4) + 0.3;
        // double speed = 40*Math.pow((slowMo-0.52), 6) + 0.2;

        double drive_y = y * Math.cos(Math.toRadians(robot_heading)) + x * Math.sin(Math.toRadians(robot_heading));
        double drive_x = -y * Math.sin(Math.toRadians(robot_heading)) + x * Math.cos(Math.toRadians(robot_heading));
        double joystick_magnitude = Math.sqrt((x*x) + (y*y));
        double scalar = 0;
        if (drive_x != 0 && drive_y != 0)
        {
            scalar = joystick_magnitude/(Math.max(Math.abs(drive_x), Math.abs(drive_y)));
        }
        drive_x = drive_x * scalar;
        drive_y = drive_y * scalar;
        driveController.driveMotors(-drive_y, turn, drive_x, speed);
        telemetry.update();
    }

    public double whichTurnMode(double turnToHeading, double turnBasic, boolean whichTurn, double time)
    {
        if (whichTurn && lastChanged < time)
        {
            flag = !flag;
            lastChanged = time + 0.25;
        }

        if (flag)
        {
            return turnToHeading;
        }
        else
        {
            return turnBasic;
        }
    }



}


