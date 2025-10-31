package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Misc.CsvLogger;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Hood;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Intake;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Shooter;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Lift;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Transfer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ActionsBackpack
{
    private static final Logger log = LoggerFactory.getLogger(ActionsBackpack.class);
    Shooter shooter;
    Intake intake;
    Lift lift;
    Hood hood;
    Transfer transfer;
    FireControl fireControl;
    ElapsedTime time;

    CsvLogger logger;

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

        logger = CsvLogger.getInstance();
        logger.start("robot_data");

        // Write CSV header
        logger.log("power,vel,hood angle");
    }


//    public Action fireball(double velocity)
//    {
//
//        return new Action()
//        {
//            private boolean initialized = false;
//            private double[] shooterParameters;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket)
//            {
//
//                shooterParameters = fireControl.firingSuite(velocity);
//                shooter.driveToVelocity(shooterParameters[1]);
//                hood.driveToAngleTarget(shooterParameters[0]);
//
//
//                double t = time.seconds();
//                double power = shooter.getPower();
//                double vel = shooter.getVelocity();
//                double hoodAngle = 0;
//                logger.log(String.format("%.3f,%.3f,%.3f,%.3f", t, power, vel, hoodAngle));
//
//                return !shooter.isBusy();
//            }
//        };
//    }

public Action fireball(double velocity)
{

    return new Action()
    {
        private boolean initialized = false;
        private double[] shooterParameters;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket)
        {
            shooterParameters = fireControl.firingSuite(velocity);
            shooter.driveToVelocity(shooterParameters[1]);
            hood.driveToAngleTarget(shooterParameters[0]);


            double t = time.seconds();
            double power = shooter.getPower();
            double vel = shooter.getVelocity();
            double hoodAngle = 0;
            logger.log(String.format("%.3f,%.3f,%.3f,%.3f", t, power, vel, hoodAngle));

            return !shooter.isBusy();
        }
    };
}

    public Action trigger(double position)
    {
        return new Action()
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                if (!initialized)
                {
                    initialized = true;
                    lift.driveServo(position);
                }

                return false;
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
                    intake.setPower(speed);
                    transfer.driveServo(speed);
                    initialized = true;
                }

                return false;
            }
        };
    }
}
