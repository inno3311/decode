package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;


@TeleOp(name = "prototype")
public class PrototypeRobot extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Transfer transfer;

    DriveController driveController;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        transfer = new Transfer(this);

        driveController = new DriveController(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            intake.toggleDrive(1, gamepad1.a);
            shooter.toggleDrive(1, gamepad1.b);
            hood.driveAngle(gamepad1.dpad_up, gamepad1.dpad_down);
            transfer.driveServo(0, gamepad1.right_trigger> 0.25);
            transfer.driveServo(1, gamepad1.right_bumper);

            driveController.gamepadController(gamepad1);
        }

    }
}
