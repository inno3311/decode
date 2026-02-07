package org.firstinspires.ftc.teamcode.Outreach.Nessie.genericOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Outreach.Nessie.IMU.IMUControl;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.fieldCentric.CentricDrive;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.fieldCentric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.roadrunner.MecanumDrive;
@TeleOp(name = "Nessie Outreach", group = "FieldCentric")
//@Disabled
public class CentricCommand extends OpMode
{
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMUControl imu;


    @Override
    public void init()
    {
        drive = new MecanumDrive(hardwareMap, null);
        imu = new IMUControl(hardwareMap, telemetry);
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);
    }

    @Override
    public void loop()
    {
        centricDrive.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                imu.getAngle(),
                gamepad1.right_trigger,
//                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                gamepad1.right_stick_x
        );
        if (gamepad1.dpad_up)
        {
            imu.resetAngle();
        }
    }
}
