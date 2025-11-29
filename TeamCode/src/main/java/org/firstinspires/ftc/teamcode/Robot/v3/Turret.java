package org.firstinspires.ftc.teamcode.Robot.v3;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Parents.CRServoParent;

public class Turret extends CRServoParent
{
    Encoder encoder;
    AprilTagLocalizer aprilTagLocalizer;
    final double ticksPerDegree = 8192 / ((12.55/0.925) * 360); // 1.677
    final double rightBoundInTicks = ticksPerDegree * 90;
    final double leftBoundInTicks = ticksPerDegree * -90;
    double target = 0;
    public Turret(AprilTagLocalizer aprilTagLocalizer, LinearOpMode opMode)
    {
        super("turret", opMode);
        encoder = new OverflowEncoder(new RawEncoder(opMode.hardwareMap.get(DcMotorEx.class, "intake")));
        this.aprilTagLocalizer = aprilTagLocalizer;
    }

//    public void trackTarget(double target, double robotHeading)
//    {
//        double error = 0;
//        double turretHeading = robotHeading + turretHeading();
//
//        if (turretHeading > 360)
//        {
//            turretHeading -= 360;
//        }
//
//        if (target == 20)
//        {
//            if (turretHeading > 315)
//            {
//                turretHeading -= 360;
//            }
//            error = 135 - turretHeading;
//        }
//        else if (target == 24)
//        {
//            if (turretHeading < 45)
//            {
//                turretHeading += 360;
//            }
//            error = 225 - turretHeading;
//        }
//
//        if (error < 0)
//        {
//            if ((encoder.getPositionAndVelocity().position * ticksPerDegree) > leftBoundInTicks)
//            {
//                driveServo(-(Math.pow(1.1, Math.abs(error)) - 1));
//            }
//            else
//            {
//                driveServo(0);
//            }
//        }
//        else if (error > 0)
//        {
//            if ((encoder.getPositionAndVelocity().position * ticksPerDegree) < rightBoundInTicks)
//            {
//                driveServo(Math.pow(1.1,error)-1);
//            }
//            else
//            {
//                driveServo(0);
//            }
//        }
//        else
//        {
//            driveServo(0);
//        }
//    }

    public void trackTarget(Gamepad gamepad)
    {
        if (gamepad.dpad_up)
        {
            target = 24;
        }
        else if (gamepad.dpad_down)
        {
            target = 20;
        }

        if (!(aprilTagLocalizer.getDetectionID() == 20 || aprilTagLocalizer.getDetectionID() == 24))
        {
            if (gamepad.left_stick_x < 0 && (encoder.getPositionAndVelocity().position * ticksPerDegree) < leftBoundInTicks)
            {
                driveServo(0);
            }
            else if (gamepad.left_stick_x > 0 && (encoder.getPositionAndVelocity().position * ticksPerDegree) > rightBoundInTicks)
            {
                driveServo(0);
            }
            else
            {
                driveServo(gamepad.left_stick_x);
            }
        }
        else
        {
            double error = 0;
            double turretHeading = turretHeading();

            if (turretHeading > 360)
            {
                turretHeading -= 360;
            }

            if (target == 20)
            {
                if (turretHeading > 315)
                {
                    turretHeading -= 360;
                }
                error = aprilTagLocalizer.getTagYaw() - turretHeading;
            }
            else if (target == 24)
            {
                if (turretHeading < 45)
                {
                    turretHeading += 360;
                }
                error = aprilTagLocalizer.getTagYaw() - turretHeading;
            }

            if (error < 0)
            {
                if ((encoder.getPositionAndVelocity().position * ticksPerDegree) > leftBoundInTicks)
                {
//                    driveServo(-(Math.pow(1.1, Math.abs(error)) - 1));
                }
                else
                {
//                    driveServo(0);
                }
            }
            else if (error > 0)
            {
                if ((encoder.getPositionAndVelocity().position * ticksPerDegree) < rightBoundInTicks)
                {
//                    driveServo(Math.pow(1.1, error) - 1);
                }
                else
                {
//                    driveServo(0);
                }
            }
            else
            {
//                driveServo(0);
            }
        }
    }

    private double turretHeading()
    {
        return encoder.getPositionAndVelocity().position * ticksPerDegree;
    }

    public void telemetry(Telemetry telemetry)
    {
        telemetry.addData("Turret", "Position: " + turretHeading());
    }

}
