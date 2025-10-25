package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Lift;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;

public class ActionsBackpack
{
    Shooter shooter;
    Intake intake;
    Lift lift;
    Hood hood;
    Transfer transfer;
    FireControl fireControl;
    ElapsedTime time;

    public ActionsBackpack(Shooter shooter, Intake intake, Lift lift, Hood hood, Transfer transfer, FireControl fireControl, ElapsedTime time)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.lift = lift;
        this.hood = hood;
        this.transfer = transfer;
        this.fireControl = fireControl;
        this.time = time;
        time.startTime();
    }

    public Action fireBall(double velocity)
    {
        return new Action()
        {
            private double[] shooterParameters;
            boolean flag = true;
            private double spinUpTime;
            private boolean shooterStop = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {

                if (shooter.getVelocity() < 10 && !shooterStop)
                {
                    shooterParameters = fireControl.firingSuite(velocity);
                    spinUpTime = time.seconds();
                }

                if (flag)
                {
                    hood.driveToAngleTarget(shooterParameters[0]);
                    shooter.driveToVelocity(shooterParameters[1]);
                }

                if (spinUpTime + 4 < time.seconds())
                {
                    lift.driveServo(0.7);
                }
                if (spinUpTime + 4.5 < time.seconds())
                {
                    shooterParameters[1] = 0;
                    shooterStop = true;
                }
                if (spinUpTime + 5 < time.seconds())
                {
                    lift.driveServo(1);
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

    public Action transferBall(double speed)
    {
        return new Action()
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                if (!initialized)
                {
                    transfer.driveServo(speed);
                    initialized = true;
                }

                return false;
            }
        };
    }
}
