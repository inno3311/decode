package org.firstinspires.ftc.teamcode.Robot.v3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.algorithms.TurretPID;

@Config
public class Turret
{
    public static class Params
    {
        public static double P = 0.0075;
        public static double I = 0;
        public static double D = 0.0175;
    }

    public static Params pid = new Params();

    DcMotorEx turret;
    final double TICKS_PER_DEGREE  = 5.5; // (Small gear 17 teeth, Big gear 100: 0.17) (Ticks per rotation 384.5) (360/(0.17 * 384.5))
    TurretPID turretPID = new TurretPID(Params.P, Params.I, Params.D);
    //POWER: 0.25 0.0125, 0, 0.000125    POWER: 0.35 P: 0.0055 D: 0.0005   POWER: 0.5 P: 0.0055  D: 0.01    POWER 0.75 P: 0.00575 D: 0.015  POWER 1 P: 0.0075 D: 0.0175
    FtcDashboard dashboard;
    Telemetry telemetry;
    double target = 0;

    public int RED_TARGET_X = -62;
    public int RED_TARGET_Y = 62;
    public int BLUE_TARGET_X = -55;
    public int BLUE_TARGET_Y = -55;

    boolean team;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry)
    {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);

        team = false;

        this.telemetry = telemetry;
    }

//    public void trackGoal(double turretFacing, Pose2d robotPose, boolean team)
//    {
//        double x;
//        double y;
//
//        if (team) // Blue
//        {
//            x = -55 - robotPose.position.x;
//            y =  55 - robotPose.position.y;
//        }
//        else // Red
//        {
//            x = -55 - robotPose.position.x;
//            y = -55 - robotPose.position.y;
//        }
//
//        // Absolute field angle to target
//        double targetAngleDeg = Math.toDegrees(Math.atan2(y,x));
//        //error = normalizeDegrees(targetAngleDeg) - Math.toDegrees(turretFacing);
//        error = (targetAngleDeg) - Math.toDegrees(turretFacing);
//
//        error = normalizeDegrees(error);
//        // Convert to robot-centric turret angle
//        //double turretAngleDeg = targetAngleDeg - robotHeadingDeg;
//
//        double power = turretPID.calculate(error * TICKS_PER_DEGREE, turret.getCurrentPosition());
//        power = Math.max(-0.25, Math.min(0.25, power));
//
//        turret.setPower(power);
//
//        telemetry.addData("Turret Error", error);
//
//    }

    public void stop()
    {
        turret.setPower(0);
    }

    public double turretAngleToFixedTarget(double robotX, double robotY, double robotHeadingDeg)
    {
        // Fixed field target

        double targetX;
        double targetY;
        if (team) // Blue
        {
            targetX = -60.0;
            targetY = -60.0;
        }
        else
        {
            targetX = -60.0;
            targetY =  60.0;
        }

        // Vector from robot to target
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Absolute field angle to target
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to robot-centric turret angle
        double turretAngleDeg = targetAngleDeg - robotHeadingDeg;

        double noralizedDeg = normalizeDegrees(turretAngleDeg);

        if (Math.abs(noralizedDeg) <= 88)
        {
            double power = turretPID.calculate(-noralizedDeg * TICKS_PER_DEGREE, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
        }

        return noralizedDeg * TICKS_PER_DEGREE;
    }
    public double normalizeDegrees(double angle)
    {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    void setTeam(boolean isBlue)
    {
        team = isBlue;
    }

    public double getPosition()
    {
        return turret.getCurrentPosition();
    }

    public void tuning(Gamepad gamepad)
    {
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);

        turretPID.setPID(Params.P, Params.I, Params.D);

        if (gamepad.y)
        {
            target = 0;
            double power = turretPID.calculate(target, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
        }
        else if (gamepad.b)
        {
            target = -100;
            double power = turretPID.calculate(target, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
        }
        else if (gamepad.a)
        {
            target = 300;
            double power = turretPID.calculate(target, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
        }
        else if (gamepad.x)
        {
            target = -150;
            double power = turretPID.calculate(target, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
        }

        packet.put("target", target);
        packet.put("Current Position", turret.getCurrentPosition());
        packet.put("Zero", 0);
    }

}
