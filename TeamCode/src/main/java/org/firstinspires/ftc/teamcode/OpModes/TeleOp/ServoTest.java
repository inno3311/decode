package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode
{
    DcMotor motor;
    Servo servo;
    ElapsedTime time;
    private final double startingAngle = 30;
    private final double servoRange = 84.375;

    @Override
    public void runOpMode() throws InterruptedException
    {
        time = new ElapsedTime();
        servo = hardwareMap.servo.get("servo");

        motor = this.hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        double servoTarget = 0;
        double flagTime = 0;
        time.startTime();
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.dpad_up && flagTime < time.seconds())
            {
                flagTime = time.seconds() + 0.25;
                servoTarget += 5;
            }
            else if (gamepad1.dpad_down && flagTime < time.seconds())
            {
                flagTime = time.seconds() + 0.25;
                servoTarget -= 5;
            }

            if (gamepad1.a)
            {
                servo.setPosition(driveServoToAngle(0));
                Thread.sleep(2000);
                servo.setPosition(driveServoToAngle(90));
                Thread.sleep(2000);
                servo.setPosition(driveServoToAngle(20));
                Thread.sleep(2000);
                servo.setPosition(driveServoToAngle(70));
                Thread.sleep(2000);
                servoTarget = 45;
            }
            else
            {
                servo.setPosition(driveServoToAngle(servoTarget));
            }

            motor.setPower(gamepad1.right_stick_y);

        }
    }

    private double driveServoToAngle(double angle)
    {
//        if (angle < startingAngle)
//        {
//            return 0;
//        }
        return angle/servoRange;
    }
}

//110
//270
//5/16