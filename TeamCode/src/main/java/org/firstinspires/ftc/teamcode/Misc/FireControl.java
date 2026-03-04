package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;

public class FireControl
{
    public AprilTagLocalizer localizer;
    public Telemetry telemetry;
    private final double g = 9.8;//meter/sec

    //Fly Wheel Stats
    private final double shooterWheelRadius = 0.0508;//meter
    private final double maxVelocity = 25;//The max velocity at which we will fire the artifact in m/s
    private final double maxLaunchAngle = 90;
    private final double minimumAngle = 65;



    public FireControl(AprilTagLocalizer localizer, Telemetry telemetry)
    {
        this.localizer = localizer;
        this.telemetry = telemetry;
    }

    /**
     * @param flightTime time of desired flight in seconds
     * @return the velocity at which to fire in the first element and the angle in the second
     */
//    public double[] timeLaunch(double flightTime)
//    {
//        double velocity = velocityMag(velocityX(flightTime), velocityY(flightTime));
//        if (velocity > maxVelocity) {velocity = maxVelocity;}
//        double angle = calculateSteeperAngle(velocity, );
//
//        return new double[] {velocity, angle};
//    }

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
        try{
            return (localizer.getTagY() + 4.9 * Math.pow(time, 2)) / time;
        } catch(Exception e){
            return(9);
        }
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
    private double calculateSteeperAngle(double velocity, double targetRange)
    {
        double targetZ = 1.3;

        double numeratorY = g * Math.pow((2 * targetRange), 2);
        double numeratorZ = 4 * (targetZ) * Math.pow(velocity, 2);
        double numeratorRoot = Math.sqrt(Math.pow(velocity, 4) - g * (numeratorY + numeratorZ));
        double numerator = Math.pow(velocity, 2) + numeratorRoot;
        double denominator = g * 2 * targetRange;
        double launchAngle = Math.atan(numerator/denominator);

        return Math.toDegrees(launchAngle);
    }

    /**
     * @targetX The plane distance to the target in meters
     * @targetY The height of target in meters
     * @param velocity The exit velocity of the artifact from the launcher in meters/second
     * @return The angle of launch in degrees
     */
    private double calculateShallowerAngle(double velocity, double targetRange)
    {
        double targetZ = 1.4;

        double numeratorY = g * Math.pow((2 * targetRange), 2);
        double numeratorZ = 4 * (targetZ) * Math.pow(velocity, 2);
        double numeratorRoot = Math.sqrt(Math.pow(velocity, 4) - g * (numeratorY + numeratorZ));
        double numerator = Math.pow(velocity, 2) - numeratorRoot;
        double denominator = g * 2 * targetRange;
        double launchAngle = Math.atan(numerator/denominator);

        return Math.toDegrees(launchAngle);
    }

    /**
     * @targetX The plane distance to the target in meters
     * @targetY The height of target in meters
     * @param angle The angle at which the ball exits the shooter in degrees
     */
    private double calculateVelocity(double angle, double targetRange)
    {
        double targetZ = 1.5;

        double numeratorParth = Math.pow((Math.tan(Math.toRadians(angle))), 2) + 1;
        double numerator = -g * Math.pow(2 * targetRange,2) * numeratorParth;
        double denominator = 4 * (targetZ - targetRange * Math.tan(Math.toRadians(angle)));
        double launchVelocity = Math.sqrt(numerator / denominator);

        return launchVelocity;
    }

    /**
     * @param velocity at which the tha ball will be launched.
     * @return the motor RPM at which to fire the ball
     * I have no idea how this math is right but I have no care in the world. It does
     */
    public double targetMotorVelocity(double velocity)
    {

        double motorVelocity = ((velocity * 1.4)/(2 * Math.PI*shooterWheelRadius)) * 28;

        telemetry.addData("Target Motor Velocity", motorVelocity);
        return motorVelocity;
    }

    public double[] firingSuite(Pose2d robotPose, boolean team)
    {
        double velocity;
        double targetAngle;
        double targetRange;

        try
        {
            if (team) // Blue
            {
                double x = -63 - robotPose.position.x;
                double y = -63 - robotPose.position.y;
                targetRange = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            }
            else // Red
            {
                double x = -63 - robotPose.position.x;
                double y = 63 - robotPose.position.y;
                targetRange = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            }

            // Convert from inches to meters
            targetRange *= 0.0254;
        }
        catch (Exception e)
        {
            targetRange = 2;
        }

        if (targetRange > 2.65)
        {
            targetAngle = 65;
            velocity = calculateVelocity(65, targetRange)+1;
        }
        else
        {
            velocity = (targetRange - 2.2) + 10;
            targetAngle = calculateSteeperAngle(velocity, targetRange)+3;
        }

        telemetry.addData("Target Range", targetRange);
        return new double[] {maxLaunchAngle - targetAngle, targetMotorVelocity(velocity)};
    }

    public double[] firingSuite(Pose2d robotPose, PoseVelocity2d velocity2d, boolean team)
    {
        double velocity = 0;
        double velocityOffset = 0;
        double targetAngle;
        double targetRange;

        try
        {
            if (team) // Blue
            {
                double x = -63 - robotPose.position.x;
                double y = -63 - robotPose.position.y;
                targetRange = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
                velocityOffset = (velocity2d.linearVel.x * Math.cos((5*Math.PI/4)-robotPose.heading.toDouble()) * 0.0254) + (velocity2d.linearVel.y * Math.sin((5*Math.PI/4)-robotPose.heading.real) * 0.0254);
            }
            else // Red
            {
                double x = -63 - robotPose.position.x;
                double y = 63 - robotPose.position.y;
                targetRange = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
                velocityOffset = (velocity2d.linearVel.x * Math.cos((3*Math.PI/4)-robotPose.heading.toDouble()) * 0.0254) + (velocity2d.linearVel.y * Math.sin((3*Math.PI/4)-robotPose.heading.real) * 0.0254);
            }

            // Convert from inches to meters
            targetRange *= 0.0254;
        }
        catch (Exception e)
        {
            targetRange = 2;
        }

        if (targetRange > 2.65)
        {
            targetAngle = 65;
            velocity = calculateVelocity(65, targetRange)+1.5;
        }
        else
        {
            velocity += (targetRange - 2.2) + 10;
            targetAngle = calculateSteeperAngle(velocity, targetRange);
            if (velocityOffset < 0.1)
            {
                velocity -= (velocityOffset - 1);
            }
            else if (velocityOffset > 0.1)
            {
                velocity -= velocityOffset;
            }
        }


        telemetry.addData("Target Range", targetRange);
        return new double[] {maxLaunchAngle - targetAngle, targetMotorVelocity(velocity), velocity};
    }

    //    public double curVelocity(double motorVelocity)
//    {
//        double velocity = ((motorVelocity/28)*(2*Math.PI*shooterWheelRadius)) / 1.5;
//
//        telemetry.addData("Velocity", motorVelocity);
//        return velocity;
//    }

}
