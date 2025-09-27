package org.firstinspires.ftc.teamcode.Misc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;

public class FireControl
{
    AprilTagLocalizer localizer;
    Telemetry telemetry;
    private final double g = 9.8;

    //Fly Wheel Stats
    private final double shooterWheelRadius = 2;//in
    private final double projectileWeight = 1;//g
    private final double shooterWheelGearRatio = 2.2857;
    private final double motorMaxRPM = 6000;
    private final double shooterMOI = 1.164;//
    private final double maxVelocity = 25;//The max velocity at which we will fire the artifact in m/s



    public FireControl(AprilTagLocalizer localizer, Telemetry telemetry)
    {
        this.localizer = localizer;
        this.telemetry = telemetry;
    }

    /**
     * @param flightTime time of desired flight in seconds
     * @return the velocity at which to fire in the first element and the angle in the second
     */
    public double[] timeLaunch(double flightTime)
    {
        double velocity = velocityMag(velocityX(flightTime), velocityY(flightTime));
        if (velocity > maxVelocity) {velocity = maxVelocity;}
        double angle = calculateAngle(velocity);

        return new double[] {velocity, angle};
    }

    /**
     * @param time of flight desired
     * @return the velocity in the X-Direction
     */
    private double velocityX(double time)
    {
        return localizer.getTagX() / time;
    }

    /**
     * @param time of flight desired
     * @return the velocity in the Y-Direction
     */
    private double velocityY(double time)
    {
        return (localizer.getTagY() + 4.9 * Math.pow(time, 2)) / time;
    }

    /**
     * @param velocityX velocity in X-Direction
     * @param velocityY velocity in Y-Direction
     * @return The velocity at which to launch
     */
    private double velocityMag(double velocityX, double velocityY)
    {
        return Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityY, 2));
    }

//    /**
//     * @param velocityX velocity in X-Direction
//     * @param velocityY velocity in Y-Direction
//     * @return The angle of launch to hit target
//     */
//    private double calculateAngle(double velocityX, double velocityY)
//    {
//        return Math.toDegrees(Math.atan(velocityX/velocityY));
//    }


    /**
     * @return Time of ball flight in seconds
     */
    public double flightTime(double velocity, double angle)
    {
        return (2 * velocity * Math.sin(angle)) / 9.8;
    }

    /**
     * @targetX The plane distance to the target in meters
     * @targetY The height of target in meters
     * @param velocity The exit velocity of the artifact from the launcher in meters/second
     * @return The angle of launch in degrees
     */
    public double calculateAngle(double velocity)
    {
        double targetX = localizer.getTagX();
        double targetY = localizer.getTagY();

        double numeratorX = g * Math.pow((2 * targetX), 2);
        double numeratorY = 4 * (targetY) * Math.pow(velocity, 2);
        double numeratorRoot = Math.sqrt(Math.pow(velocity, 4) - g * (numeratorX + numeratorY));
        double numerator = Math.pow(velocity, 2) + numeratorRoot;
        double denominator = g * 2 * targetX;
        double launchAngle = Math.atan(numerator/denominator);

        telemetry.addData("Launch Angle", launchAngle);
        return Math.toDegrees(launchAngle);
    }

    /**
     * @targetX The plane distance to the target in meters
     * @targetY The height of target in meters
     * @param angle The angle at which the ball exits the shooter in degrees
     */
    public double calculateVelocity(double angle)
    {
        double targetX = localizer.getTagX();
        double targetY = localizer.getTagY();

        double numeratorParth = Math.pow((Math.tan(Math.toRadians(angle))), 2) + 1;
        double numerator = -g * targetX * numeratorParth;
        double denominator = 2 * (targetY - targetX * Math.tan(Math.toRadians(angle)));
        double launchVelocity = Math.sqrt(numerator / denominator);


        telemetry.addData("Launch Velocity", launchVelocity);
        return launchVelocity;
    }

    /**
     * @param velocity at which the tha ball will be launched.
     * @return the motor RPM at which to fire the ball
     */
    public double targetMotorRPM(double velocity)
    {
        double speedTransferPercentage = 20 * shooterMOI / (7 * projectileWeight * Math.pow((shooterWheelRadius / 2), 2) + 40 * shooterMOI);
        double wheelSurfaceSpeed = velocity / speedTransferPercentage;
        double shooterWheelRPM = wheelSurfaceSpeed / shooterWheelRadius;
        double motorRPM = shooterWheelRPM/shooterWheelGearRatio;

        telemetry.addData("MotorRPM", motorRPM);
        return motorRPM;
    }

}
