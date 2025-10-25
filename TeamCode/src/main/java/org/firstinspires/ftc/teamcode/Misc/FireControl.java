package org.firstinspires.ftc.teamcode.Misc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;

public class FireControl
{
    AprilTagLocalizer localizer;
    Telemetry telemetry;
    private final double g = 9.8;//meter/sec

    //Fly Wheel Stats
    private final double shooterWheelRadius = 0.0508;//meter
    private final double maxVelocity = 25;//The max velocity at which we will fire the artifact in m/s
    private final double maxLaunchAngle = 65;



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
        double angle = calculateSteeperAngle(velocity);

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
    private double calculateSteeperAngle(double velocity)
    {
        double targetY = localizer.getTagY() * 2.54 / 100;
        double targetZ = 1.1;

        double numeratorY = g * Math.pow((2 * targetY), 2);
        double numeratorZ = 4 * (targetZ) * Math.pow(velocity, 2);
        double numeratorRoot = Math.sqrt(Math.pow(velocity, 4) - g * (numeratorY + numeratorZ));
        double numerator = Math.pow(velocity, 2) + numeratorRoot;
        double denominator = g * 2 * targetY;
        double launchAngle = Math.atan(numerator/denominator);

        return Math.toDegrees(launchAngle);
    }

    /**
     * @targetX The plane distance to the target in meters
     * @targetY The height of target in meters
     * @param velocity The exit velocity of the artifact from the launcher in meters/second
     * @return The angle of launch in degrees
     */
    private double calculateShallowerAngle(double velocity)
    {
        double targetY = localizer.getTagY() * 2.54 / 100;
        double targetZ = 1.1;

        double numeratorY = g * Math.pow((2 * targetY), 2);
        double numeratorZ = 4 * (targetZ) * Math.pow(velocity, 2);
        double numeratorRoot = Math.sqrt(Math.pow(velocity, 4) - g * (numeratorY + numeratorZ));
        double numerator = Math.pow(velocity, 2) - numeratorRoot;
        double denominator = g * 2 * targetY;
        double launchAngle = Math.atan(numerator/denominator);

        return Math.toDegrees(launchAngle);
    }

    /**
     * @targetX The plane distance to the target in meters
     * @targetY The height of target in meters
     * @param angle The angle at which the ball exits the shooter in degrees
     */
    private double calculateVelocity(double angle)
    {
        double targetY = localizer.getTagY() * 2.54 / 100;
        double targetZ = 1.1;

        double numeratorParth = Math.pow((Math.tan(Math.toRadians(angle))), 2) + 1;
        double numerator = -g * targetY * numeratorParth;
        double denominator = 2 * (targetZ - targetY * Math.tan(Math.toRadians(angle)));
        double launchVelocity = Math.sqrt(numerator / denominator);

        return launchVelocity;
    }

    /**
     * @param velocity at which the tha ball will be launched.
     * @return the motor RPM at which to fire the ball
     * I have no idea how this math is right but I have no care in the world. It does
     */
    private double targetMotorVelocity(double velocity)
    {

        double motorVelocity = (velocity/(Math.PI * shooterWheelRadius)) * 28;

        telemetry.addData("Target Motor Velocity2", motorVelocity);
        return motorVelocity;
    }

    public double[] firingSuite(double velocity)
    {
        double targetAngle;

        double targetY = localizer.getTagY() * 2.54 / 100;
        telemetry.addData("Plan Distance", targetY);

        if (targetY > 2.5)
        {
            targetAngle = calculateShallowerAngle(velocity);
        }
        else if (maxLaunchAngle > calculateSteeperAngle(velocity))
        {
            targetAngle = calculateSteeperAngle(velocity);
        }
        else
        {
            targetAngle = maxLaunchAngle;
            velocity = calculateVelocity(maxLaunchAngle);
        }

        telemetry.addData("Target Velocity", velocity);
        telemetry.addData("Taget Angle", targetAngle);

        return new double[] {targetAngle, targetMotorVelocity(velocity)};
    }

}
