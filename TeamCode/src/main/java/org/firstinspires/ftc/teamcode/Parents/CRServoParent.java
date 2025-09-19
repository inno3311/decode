package org.firstinspires.ftc.teamcode.Parents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Logging;

public class CRServoParent
{
    private CRServo servo;
    private String servoName;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    private CRServoParent(LinearOpMode opMode)
    {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }

    protected CRServoParent(String servoName, LinearOpMode opMode)
    {
        this(opMode);

        try
        {
            this.servoName = servoName;
            this.servo = hardwareMap.crservo.get(servoName);

        }
        catch (IllegalArgumentException e)
        {
            Logging.log("%s not found in Hardware Map",servoName);
            telemetry.addData("Exception:", "%s not found in Hardware Map",servoName);
            telemetry.update();
        }

    }

    protected void stop()
    {
        servo.setPower(0);
    }
    protected void driveForward()
    {
        servo.setPower(1);
    }

    protected void driveForwardBoolean(boolean flag)
    {
        driveForward();
    }

    protected void driveBackward()
    {
        servo.setPower(-1);
    }

    protected void driveBackwardBoolean(boolean flag)
    {
        driveBackward();
    }

    protected void driveServo(boolean direction)
    {
        if (direction)
        {
            driveForward();
        }
        else if (!direction)
        {
            driveBackward();
        }
    }

    protected void driveServoBoolean(boolean forward, boolean backward)
    {
        if (forward)
        {
            driveForward();
        }
        else if (backward)
        {
            driveBackward();
        }
//        else
//        {
//            stop();
//        }
    }

    public Action action(double target)
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                driveForward();
                return false;
            }
        };
    }


}
