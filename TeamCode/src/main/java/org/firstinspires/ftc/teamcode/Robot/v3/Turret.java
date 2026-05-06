package org.firstinspires.ftc.teamcode.Robot.v3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.algorithms.TurretPID;
import org.opencv.core.Mat;

@Config
public class Turret
{
    public static class Params
    {
        public static double P = 0.0075;
        public static double I = 0;
        public static double D = 0.005;
    }

    public static Params pid = new Params();

    DcMotorEx turret;
    final double TICKS_PER_DEGREE  = 6.222222; // (Small gear 17 teeth, Big gear 100: 0.17) (Ticks per rotation 384.5) (360/(0.17 * 384.5))
    TurretPID turretPID = new TurretPID(Params.P, Params.I, Params.D);
    //POWER: 0.25 0.0125, 0, 0.000125    POWER: 0.35 P: 0.0055 D: 0.0005   POWER: 0.5 P: 0.0055  D: 0.01    POWER 0.75 P: 0.00575 D: 0.015  POWER 1 P: 0.0125 D: 0.0175
    TouchSensor turretLimit;
    FtcDashboard dashboard;
    Telemetry telemetry;
    double target = 0;

    double targetX;
    double targetY;

    boolean isBlue;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, boolean team)
    {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        isBlue = team;

        turretLimit = hardwareMap.get(TouchSensor.class, "turretLimit");

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);

        this.telemetry = telemetry;

        //boolean team = false;
        if (isBlue) // Blue
        {
            targetX = -63.0;
            targetY = -63.0;
        }
        else
        {
            //targetX = -54.0;  //Use for auto
            targetX = -63.0;
            targetY =  63.0;
        }
    }

    public double turretAngleToFixedTarget(double robotX, double robotY, double robotHeadingDeg, boolean team, double offset)
    {
        // Fixed field target

        turretPID.setPID(Params.P, Params.I, Params.D);


        if (team) // Blue
        {
            targetX = -65.0;
            targetY = -65.0;
        }
        else
        {
            //targetX = -50.0;  //Use for auto
            targetX = -65.0;
            targetY =  65.0;
        }

        // Vector from robot to target
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Absolute field angle to target
        double targetAngleDeg = Math.abs(180 + Math.toDegrees(Math.atan(dy/dx)));

        // Convert to robot-centric turret angle
        double turretAngleDeg = targetAngleDeg - (robotHeadingDeg + offset);

        double noralizedDeg = normalizeDegrees(turretAngleDeg);

        double targetInTicks = -noralizedDeg * TICKS_PER_DEGREE;

        if (targetInTicks > 0 && targetInTicks < 1115)
        {
            double power = turretPID.calculate(targetInTicks, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
            //turret.setPower(0.3);
        }
        else
        {
            turret.setPower(0);
        }

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);
        packet.put("target", targetInTicks);
        packet.put("__turret Current Position", turret.getCurrentPosition());
        packet.put("__turret Heading turretAngleDeg", turretAngleDeg);
        packet.put("__turret Heading noralizedDeg", noralizedDeg);
        //packet.put("Zero", 0);

        return noralizedDeg;
    }

    public double turretAngleToFixedTarget(double robotX, double robotY, double robotHeadingDeg, PoseVelocity2d velocity2d, double shooterVelocity, boolean team, double offset)
    {
        // Fixed field target
        turretPID.setPID(Params.P, Params.I, Params.D);

        double trigVariable;
        if (team) // Blue
        {
            targetX = -60.0;
            targetY = -60.0;
            trigVariable = (5*Math.PI)/4;
        }
        else
        {
            //targetX = -50.0;  //Use for auto
            targetX = -60.0;
            targetY =  60.0;
            trigVariable = (3*Math.PI)/4;
        }

//        double robotVelocityMagnitude = Math.sqrt((Math.pow(velocity2d.linearVel.x * Math.sin(trigVariable),2) * 0.0254)
//                + (Math.pow(velocity2d.linearVel.y*Math.cos(trigVariable),2) * 0.0254));
//
//        double velocityOffsetAngle = 0;
//        try
//        {
//            velocityOffsetAngle = Math.toDegrees(Math.atan2(robotVelocityMagnitude,shooterVelocity));
//        }
//        catch (Exception e) {velocityOffsetAngle = 0;}

        double robotVelocityVector;
        double velocityX = Math.round(velocity2d.linearVel.x*0.0254/2.5);
        double velocityY = Math.round(velocity2d.linearVel.y*0.0254/2.5);
        try
        {
            robotVelocityVector = Math.sqrt(Math.pow(velocityX,2)
                    + Math.pow(velocityY,2)) * Math.sin(trigVariable-Math.atan2(velocityY,velocityX));
        }
        catch (Exception e)
        {
            robotVelocityVector = 0;
        }
        double velocityOffsetAngle = Math.round(Math.toDegrees(Math.atan2(robotVelocityVector,shooterVelocity)));

        // Vector from robot to target
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Absolute field angle to target
        double targetAngleDeg = Math.abs(180 + Math.toDegrees(Math.atan(dy/dx) - velocityOffsetAngle));

        // Convert to robot-centric turret angle
        double turretAngleDeg = targetAngleDeg - (robotHeadingDeg + offset);

        double noralizedDeg = normalizeDegrees(turretAngleDeg);

        double targetInTicks = -noralizedDeg * TICKS_PER_DEGREE;

        if (targetInTicks > 0 && targetInTicks < 1115)
        {
            double power = turretPID.calculate(targetInTicks, turret.getCurrentPosition());
            power = Math.max(-1, Math.min(1, power));
            turret.setPower(power);
            //turret.setPower(0.3);
        }
        else
        {
            turret.setPower(0);
        }

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);
        packet.put("target", targetInTicks);
        packet.put("turret Current Position", turret.getCurrentPosition());
        //packet.put("Zero", 0);

        return noralizedDeg;
    }

    public double normalizeDegrees(double angle)
    {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public double getPosition()
    {
        return turret.getCurrentPosition();
    }

    public void driveToPosition(double target)
    {
        double power = turretPID.calculate(target, turret.getCurrentPosition());
        power = Math.max(-1, Math.min(1, power));
        turret.setPower(power);
    }

    public void zero(Gamepad gamepad)
    {
        if (!turretLimit.isPressed())
        {
            if (gamepad.left_stick_x != 0)
            {
                turret.setPower(gamepad.left_stick_x/3);
            }
            else
            {
                turret.setPower(-0.25);  //change for reset speed.
            }
        }
        else if (gamepad.left_stick_x == 0)
        {
            turret.setPower(0);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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

    public void stop()
    {
        turret.setPower(0);
    }

    public void trimX(int value)
    {
        targetX = targetX + value;
    }

    public void trimY(int value)
    {
        targetY = targetY + value;
    }

    public double getTargetX()
    {
        return targetX;
    }

    public double getTargetY()
    {
        return targetY;
    }

    public void trimRight()
    {
        int pos = turret.getCurrentPosition();
        turret.setTargetPosition(pos +5);
    }

    public void trimLeft()
    {
        int pos = turret.getCurrentPosition();
        turret.setTargetPosition(pos - 5);
    }

}
