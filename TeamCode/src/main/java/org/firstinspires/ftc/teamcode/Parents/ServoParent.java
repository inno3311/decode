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
    double minPosition, maxPosition;

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

    protected ServoParent(String servoName, double minPosition, double maxPosition, LinearOpMode opMode)
    {
        this(opMode);

        try
        {
            this.servoName = servoName;
            servo = hardwareMap.servo.get(servoName);

            this.minPosition = minPosition;
            this.maxPosition = maxPosition;
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
//        if (servo.getPosition() != target)
//        {
            servo.setPosition(target);
//        }
//        else
//        {
//            servo.setPosition(servo.getPosition());
//        }
    }

    protected void driveServo(double target, boolean argument)
    {
        if (argument)
        {
            driveServo(target);
        }
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


    protected void telemetry()
    {
        telemetry.addData(servoName, "minPosition: %.2f\n" +
                "\tmaxPosition: %.2f", minPosition, maxPosition);
    }

}
