package org.firstinspires.ftc.teamcode.Robot.CommonFeatures;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parents.ServoParent;

public class Led extends ServoParent
{
    public Led(LinearOpMode opMode)
    {
        super("led", 0, opMode);
    }

    @Override
    public void driveServo(double target)
    {
        super.driveServo(target);
    }

    @Override
    public double getPosition()
    {
        return super.getPosition();
    }

    public void setOff() {super.driveServo(0);}

    public void setRed() {super.driveServo(.277);}

    public void setOrange() {super.driveServo(.333);}

    public void setGreen() {super.driveServo(.5);}

    public void setYellow() {super.driveServo(.388);}

    public void setBlue() {super.driveServo(.611);}

    public void setViolet() {super.driveServo(.722);}

    public void setWhite() {super.driveServo(1);}
}
