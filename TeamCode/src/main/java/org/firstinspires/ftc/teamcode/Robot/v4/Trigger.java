package org.firstinspires.ftc.teamcode.Robot.v4;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Trigger
{
    DcMotor motor;

    public Trigger(HardwareMap hardwareMap)
    {
        motor = hardwareMap.get(DcMotor.class, "trigger");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setPower(double power)
    {
        motor.setPower(power);
    }
}
