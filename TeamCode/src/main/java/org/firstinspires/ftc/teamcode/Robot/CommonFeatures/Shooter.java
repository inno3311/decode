package org.firstinspires.ftc.teamcode.Robot.CommonFeatures;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter
{
    DcMotorEx shooter;

    public static class Params
    {
        public static double F = 25;
        public static double P = 60;
        public static double I = 0;
        public static double D = 5;
    }

    public static Params PARAMS = new Params();

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 0;

    FtcDashboard dashboard;
    Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry)
    {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Params.P, 0, 0, Params.F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        this.telemetry = telemetry;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", 0);
        packet.put("Current Velocity", 0);
        packet.put("zero", 0);
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * In ticks per second
     */
    public void driveToVelocity(double targetVelocity)
    {
        shooter.setVelocityPIDFCoefficients(Params.P, Params.I, Params.D, Params.F);
        shooter.setVelocity(targetVelocity);

        double currentVelocity = shooter.getVelocity();
        double error = targetVelocity - currentVelocity;

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addLine("------------------------------");

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", targetVelocity);
        packet.put("Current Velocity", currentVelocity);
        packet.put("zero", 0);
        dashboard.sendTelemetryPacket(packet);
    }

    private void tuning(double targetVelocity, Gamepad gamepad)
    {
        if (gamepad.bWasPressed())
        {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad.dpadRightWasPressed())
        {
            Params.F += stepSizes[stepIndex];
        }
        if (gamepad.dpadLeftWasPressed())
        {
            Params.F -= stepSizes[stepIndex];
        }

        if (gamepad.dpadUpWasPressed())
        {
            Params.P += stepSizes[stepIndex];
        }
        if (gamepad.dpadDownWasPressed())
        {
            Params.P -= stepSizes[stepIndex];
        }

        shooter.setVelocityPIDFCoefficients(Params.P, Params.I, Params.D, Params.F);
        shooter.setVelocity(targetVelocity);

        double currentVelocity = shooter.getVelocity();
        double error = targetVelocity - currentVelocity;

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", Params.P);
        telemetry.addData("Tuning F", Params.F);
        telemetry.addData("Step Size",  stepSizes[stepIndex]);
        telemetry.addLine("------------------------------");

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", targetVelocity);
        packet.put("Current Velocity", currentVelocity);
        packet.put("Tuning P", Params.P);
        packet.put("Tuning F", Params.F);
        packet.put("zero", 0);
        dashboard.sendTelemetryPacket(packet);

    }

    //needed for auto stuff.
    public DcMotorEx getShooter()
    {
        return shooter;
    }
}
