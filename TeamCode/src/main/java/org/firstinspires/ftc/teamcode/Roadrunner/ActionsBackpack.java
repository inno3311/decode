package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;

public class ActionsBackpack
{
    Shooter shooter;
    Intake intake;
    Transfer transfer;
    Hood hood;
    FireControl fireControl;

    public ActionsBackpack(Shooter shooter, Intake intake, Transfer transfer, Hood hood, FireControl fireControl)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
        this.hood = hood;
        this.fireControl = fireControl;
    }

    public Action fireBall(double velocity)
    {

        return new Action()
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                boolean flag = true;
                if (!initialized)
                {
                    hood.driveToAngleTarget(65 - fireControl.calculateSteeperAngle(velocity));
                    shooter.driveToVelocity(fireControl.targetMotorVelocity(velocity));
                    initialized = true;
                }

                // I don't know if this while loop is necessary. If problems remove it
                if (shooter.getVelocity() > fireControl.targetMotorVelocity(velocity) - 100)
                {
                    transfer.driveServo(0.7);
                }

                if (transfer.getPosition() == 0.7)
                {
                    try
                    {
                        Thread.sleep(1000);
                    }
                    catch (InterruptedException e)
                    {
                        throw new RuntimeException(e);
                    }
                    transfer.driveServo(1);
                    shooter.setPower(0);
                    flag = false;
                }

                return flag;
            }
        };

    }

    public Action intakeBall(double speed)
    {

        return new Action()
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                if (!initialized)
                {
                    intake.setPower(speed);
                    initialized = true;
                }

                return !intake.isBusy();
            }
        };

    }
}
