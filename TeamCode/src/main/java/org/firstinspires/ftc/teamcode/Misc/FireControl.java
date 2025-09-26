package org.firstinspires.ftc.teamcode.Misc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import java.util.Vector;

public class FireControl
{
    AprilTagLocalizer localizer;
    Telemetry telemetry;
    private final double g = 9.8;

    //Fly Wheel Stats
    private final double shooterWheelRadius = 2;
    private final double projectileWeight = 1;
    private final double shooterWheelGearRatio = 2;
    private final double motorRPM = 6000;
    private final double shooterWheelRPM = motorRPM * shooterWheelGearRatio;
    private final double shooterMOI = 0.0001;


    public FireControl(AprilTagLocalizer localizer, Telemetry telemetry)
    {
        this.localizer = localizer;
        this.telemetry = telemetry;
    }

    /**
     *
     * @param pVelocity the exit velocity of the artifact from the launcher in meters/second
     * @param targetX The plane distance to the target in meters
     * @param targetY The height of target in meters
     */
    private double calculateAngle(double pVelocity, double targetX, double targetY)
    {
        double numeratorX = g * Math.pow((2 * targetX), 2);
        double numeratorY = 4 * (targetY) * Math.pow(pVelocity, 2);
        double numeratorRoot = Math.pow(Math.pow(pVelocity, 4) - g * (numeratorX + numeratorY), 0.5);
        double numerator = Math.pow(pVelocity, 2) + numeratorRoot;
        double denominator = g * 2 * targetX;
        double launchAngle = Math.atan(numerator/denominator);

        telemetry.addData("Launch Angle", launchAngle);
        return launchAngle;
    }

    private double pVelocity()
    {
        double wheelSurfaceSpeed = shooterWheelRPM * shooterWheelRadius;
        double speedTransferPercentage = 20 * shooterMOI / (7 * projectileWeight * Math.pow((shooterWheelRadius / 2), 2) + 40 * shooterMOI);
        double velocity = wheelSurfaceSpeed * speedTransferPercentage;
        telemetry.addData("Velocity", velocity);
        return velocity;
    }



}
