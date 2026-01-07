package org.firstinspires.ftc.teamcode.Robot.v3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.algorithms.TurretPID;

public class Turret
{
    DcMotorEx turret;
    final double TICKS_PER_DEGREE  = 5.4616; // (Small gear 54 teeth, Big gear 315: 0.1741) (Ticks per rotation 384.5) (360/(0.1741 * 384.5))
    TurretPID turretPID = new TurretPID(0.015, 0, 0.00075);
    FtcDashboard dashboard;
    Telemetry telemetry;
    double error;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry)
    {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);


        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);

        this.telemetry = telemetry;
    }

    public void trackGoal(double turretFacing, Pose2d robotPose, boolean team)
    {
        double x;
        double y;

        if (team) // Blue
        {
            x = -65 - robotPose.position.x;
            y =  60 - robotPose.position.y;
        }
        else // Red
        {
            x = -65 - robotPose.position.x;
            y = -60 - robotPose.position.y;
        }

        double goal = Math.toDegrees(Math.atan(y/x));
        error = goal - (turretFacing);


        double power = turretPID.calculate(error * TICKS_PER_DEGREE, turret.getCurrentPosition());
        power = Math.max(-0.25, Math.min(0.25, power));

        turret.setPower(power);

        telemetry.addData("Turret Current Position", turret.getCurrentPosition());
        telemetry.addData("Error", error);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current Position", turret.getCurrentPosition());
        packet.put("Target Position", error * TICKS_PER_DEGREE);
        packet.put("zero", 0);
        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);

    }

    public void stop()
    {
        turret.setPower(0);
    }

    public double getError()
    {
        return error;
    }

}
