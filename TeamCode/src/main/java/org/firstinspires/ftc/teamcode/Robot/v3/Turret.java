package org.firstinspires.ftc.teamcode.Robot.v3;

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
    final double TICK_PER_DEGREE  = 5.3778; // (Small gear 55 teeth, Big dear 316: 0.1741) (Ticks per rotation 384.5) (360/(0.1741 * 384.5))
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

    public void trackGoal(double goalAngle, Gamepad gamepad)
    {
        if (gamepad.y)
        {
            turret.setTargetPosition(500);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.5);
        }
        else if (gamepad.x)
        {
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.5);
        }

        telemetry.addData("Turret Current Position", turret.getCurrentPosition());
    }
}
