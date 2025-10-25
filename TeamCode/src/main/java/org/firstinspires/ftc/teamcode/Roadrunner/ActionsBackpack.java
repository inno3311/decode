package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public Action fireBall(double velocity, double numberOfShots)
    {
        return new Action()
        {
            private double[] shooterParameters;
            boolean flag = true;
            private double fireHold;
            private boolean shooterStop = false;
            private double shotsFired = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {

                if (shooter.getVelocity() < 10 && !shooterStop)
                {
                    shooterParameters = fireControl.firingSuite(velocity);
                    fireHold = time.seconds();
                }

                if (flag)
                {
                    hood.driveToAngleTarget(shooterParameters[0]);
                    shooter.driveToVelocity(shooterParameters[1]);
                }

                if (fireHold + 3 < time.seconds())
                {
                    transfer.driveServo(0);
                    lift.driveServo(0.7);
                }

                if (numberOfShots == shotsFired)
                {
                    shooterParameters[1] = 0;
                    shooterStop = true;
                }

                if (fireHold + 4 < time.seconds())
                {

                    lift.driveServo(1);
                    transfer.driveServo(1);

                    if (numberOfShots == shotsFired)
                    {
                        shooter.setPower(0);
                        flag = false;
                    }
                    else
                    {
                        shotsFired++;
                        fireHold = time.seconds() - 1;
                    }
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
