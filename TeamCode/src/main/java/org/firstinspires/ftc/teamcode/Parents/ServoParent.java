package org.firstinspires.ftc.teamcode.Parents;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Logging;

public class ServoParent
{
    private Servo servo;
    private String servoName;
    double servoRange;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    private ServoParent(LinearOpMode opMode)
    {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }

    protected ServoParent(String servoName, double servoRange, LinearOpMode opMode)
    {
        this(opMode);

        try
        {
            this.servoName = servoName;
            servo = hardwareMap.servo.get(servoName);

            this.servoRange = servoRange;
        }
        catch (IllegalArgumentException e)
        {
            Logging.log("%s not found in Hardware Map",servoName);
            telemetry.addData("Exception:", "%s not found in Hardware Map",servoName);
            telemetry.update();
        }

    }

    protected void driveServo(double target)
    {
            servo.setPosition(target);
    }

    protected void driveServo(double target, boolean argument)
    {
        if (argument)
        {
            driveServo(target);
        }
    }

    /**
     * @return servo position between 0 and 1
     */
    protected double getPosition()
    {
        return servo.getPosition();
    }

    /**
     * @param angle The angle you want the servo to drive to in its range (position 0 = o degrees, position 1 = max servoRange)
     * @return a value in between 0-1 to drive the servo to. Must be inputted into drive servo
     */
    protected void driveToAngleTarget(double angle)
    {
        servo.setPosition(1-(angle/servoRange));
    }

    protected double getAngle()
    {
        return servo.getPosition() * servoRange;
    }

    protected void driveAngle(boolean iIncrease, boolean iDecrease)
    {
        double angle = getAngle();
        if (iIncrease)
        {
            angle = getAngle() + 5;
        }
        else if (iDecrease)
        {
            angle = getAngle() - 5;
        }

        driveToAngleTarget(angle);
    }

    public Action action(double target)
    {
        return new Action()
        {
//            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
//                if (!initialized)
//                {
                    driveServo(target);
//                    initialized = true;
//                }

                return false;
            }
        };
    }

}
