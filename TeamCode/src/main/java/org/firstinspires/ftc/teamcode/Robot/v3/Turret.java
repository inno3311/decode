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
    TurretPID turretPID = new TurretPID(0.015, 0, 0.00075);//0.015, 0, 0.00075
    FtcDashboard dashboard;
    Telemetry telemetry;
    double error;

    public int RED_TARGET_X = -55;
    public int RED_TARGET_Y = 55;
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

        team = false   ;

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

    public double getError()
    {
        return error * TICKS_PER_DEGREE;
    }

    public double getErrorDegrees()
    {
        return error;
    }

    public double turretAngleToFixedTarget(
        double robotX,
        double robotY,
        double robotHeadingDeg
    ) {
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

        double power = turretPID.calculate(-noralizedDeg * TICKS_PER_DEGREE, turret.getCurrentPosition());
        power = Math.max(-0.35, Math.min(0.35, power));
        turret.setPower(power);

        return noralizedDeg;

    }
    public double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    void setTeam(boolean isBlue)
    {
        team = isBlue;
    }

}
