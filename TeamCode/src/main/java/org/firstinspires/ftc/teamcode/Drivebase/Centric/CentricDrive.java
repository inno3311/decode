package org.firstinspires.ftc.teamcode.Drivebase.Centric;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;

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
        driveController.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveController.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveController.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveController.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double speed = 8*Math.pow((slowMo-0.543879), 4) + 0.3;

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
//        if(drive_y == 0){
//            drive_y = 0.0001;
//        }
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


