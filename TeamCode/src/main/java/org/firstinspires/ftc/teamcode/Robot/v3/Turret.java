package org.firstinspires.ftc.teamcode.Robot.v3;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret
{
    DcMotorEx turret;
    final double TICKS_PER_DEGREE  = 5.3778; // (Small gear 55 teeth, Big gear 316: 0.1741) (Ticks per rotation 384.5) (360/(0.1741 * 384.5))
    double P = 0;
    double F = 0;
    double[] stepSizes = {10.0, 1.1, 0.1, 0.001, 0.0001};
    int stepIndex = 0;

    Telemetry telemetry;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry)
    {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
//        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        this.telemetry = telemetry;
    }

    public void trackGoal(double robotHeading, Pose2d robotPose, boolean team, Gamepad gamepad)
    {
        double error;
        double x;
        double y;

        if (team)
        {
            x = 55 - robotPose.position.x;
            y = -55 - robotPose.position.y;
        }
        else
        {
            x = 55 - robotPose.position.x;
            y = 55 - robotPose.position.y;
        }

        double goal = Math.toDegrees(Math.atan(y/x));

        error = goal;

//        if (team)
//        {
//            if (robotHeading >= -45)
//            {
//                error = goal-robotHeading;
//            }
//            else
//            {
//                error = -((goal+robotHeading)+90);
//            }
//        }
//        else
//        {
//            if (robotHeading >= 45)
//            {
//                error = goal-robotHeading;
//            }
//            else
//            {
//                error = (goal-robotHeading)-360;
//            }
//        }


        if (gamepad.right_trigger > 0.5 && Math.abs(error) <= 90)
        {
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setTargetPosition((int) (-error * TICKS_PER_DEGREE));
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.25);
        }

        telemetry.addData("Turret Current Position", turret.getCurrentPosition());
        telemetry.addData("Error", error);

    }
}
