package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Misc.CsvLogger;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v1.Transfer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Locale;

public class ActionsBackpack
{
    private static final Logger log = LoggerFactory.getLogger(ActionsBackpack.class);
    Shooter shooter;
    Intake intake;
    Trigger lift;
    Hood hood;
    Transfer transfer;
    FireControl fireControl;
    ElapsedTime time;

    private enum FireState {
        INIT,
        SPINUP,
        FIRE,
        FIRE_DOWN,
        TRANSFER_START,
        TRANSFER_STOP,
        RESET,
        DONE
    }

    private double m_targetVelocity = 0;

    CsvLogger svgLogger;

    public ActionsBackpack(Shooter shooter, Intake intake, Trigger lift, Hood hood, Transfer transfer, FireControl fireControl, ElapsedTime time)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.lift = lift;
        this.hood = hood;
        this.transfer = transfer;
        this.fireControl = fireControl;
        this.time = time;
        time.startTime();

//        svgLogger = CsvLogger.getInstance();
//        svgLogger.start("robot_data");
//
//        // Write CSV header
//        svgLogger.log("power,vel,hood angle");
    }

    public Action mezAction(double velocity, int numRounds)
    {

        return new Action()
        {
            private boolean initialized = false;
            private double[] shooterParameters;

            double fireTime;
            double transTime;
            double currentTime;

            int timesFired;

            FireState state = FireState.INIT;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                switch (state)
                {
                    case INIT:
                        timesFired = 0;
                        shooterParameters = fireControl.firingSuite(velocity);
                        m_targetVelocity = shooterParameters[1];
                        if (m_targetVelocity > 1500)
                            m_targetVelocity = 1500;
                        shooter.driveToVelocity(m_targetVelocity);
                        hood.driveToAngleTarget(15);
                        state = FireState.SPINUP;
                        packet.put("targetVel",m_targetVelocity);
                        packet.put("Angle",shooterParameters[0]);
                        packet.put("STATE","INIT");
                        break;
                    case SPINUP:

                        double vel = shooter.getVelocity();
                        boolean notAtSpeed = Math.abs(vel - m_targetVelocity) > 50;
                        if (notAtSpeed == false)
                        {
                            state = FireState.FIRE;
                        }
                        packet.put("vel",vel);
                        packet.put("STATE","SPINUP");
                        break;
                    case FIRE:
                        fireTime = time.seconds();
                        lift.driveServo(.7);
                        state = FireState.FIRE_DOWN;
                        packet.put("STATE","FIRE");
                        break;
                    case FIRE_DOWN:
                        currentTime = time.seconds();
                        if (currentTime - fireTime > 1)
                        {
                            lift.driveServo(1);
                            state = FireState.TRANSFER_START;
                            timesFired++;
                        }
                        packet.put("fireSeconds",currentTime - fireTime);
                        packet.put("STATE","FIRE");
                        break;
                    case TRANSFER_START:
                        intake.setPower(1);
                        transfer.driveServo(1);
                        transTime = time.seconds();
                        state = FireState.TRANSFER_STOP;
                        packet.put("STATE","TRANSFER");
                        break;
                    case TRANSFER_STOP:
                        if (time.seconds() - transTime > 2)
                        {
                            intake.setPower(0);
                            transfer.driveServo(0);

                            if (timesFired == numRounds)
                            {
                                state = FireState.DONE;
                            }
                            else
                            {
                                state = FireState.FIRE;
                            }
                        }
                        break;
                    case DONE:
                        {
                            shooter.driveToVelocity(0);
                            shooter.setPower(0);
                            packet.put("STATE","DONE");
                            return false;
                        }
                }

                packet.put("targetVel",m_targetVelocity);
                packet.put("Angle",shooterParameters[0]);


//                shooterParameters = fireControl.firingSuite(velocity);
//                m_targetVelocity = shooterParameters[1];
//                shooter.driveToVelocity(m_targetVelocity);
//
//                hood.driveToAngleTarget(shooterParameters[0]);
//                packet.put("1_ramp_targetVelocity", shooterParameters[1]);
//                packet.put("1_Angle", shooterParameters[0]);

//                double t = time.seconds();
//                double power = shooter.getPower();
//                double vel = shooter.getVelocity();
//                double hoodAngle = 0;

                return true;
            }
        };
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

    public Action mezRampUp(double velocity)
    {

        return new Action()
        {
            private boolean initialized = false;
            private double[] shooterParameters;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {

                shooterParameters = fireControl.firingSuite(velocity);
                m_targetVelocity = shooterParameters[1];
                shooter.driveToVelocity(m_targetVelocity);

                hood.driveToAngleTarget(shooterParameters[0]);
                packet.put("1_ramp_targetVelocity", shooterParameters[1]);
                packet.put("1_Angle", shooterParameters[0]);

//                double t = time.seconds();
//                double power = shooter.getPower();
//                double vel = shooter.getVelocity();
//                double hoodAngle = 0;

                return false;
            }
        };
    }

    public Action mezFire(double velocity)
    {


        return new Action()
        {


            private boolean initialized = false;
            private double[] shooterParameters;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                //shooter.getVelocity();
                //hood.driveToAngleTarget(shooterParameters[0]);

                double t = time.seconds();
                double power = shooter.getPower();
                double vel = shooter.getVelocity();
                double hoodAngle = 0;

                packet.put("m_targetVelocity", m_targetVelocity);
                packet.put("Velocity:", vel);
                packet.put("Time: ", t);

                boolean notAtSpeed = Math.abs(vel - m_targetVelocity) > 50;

                if (notAtSpeed == false)
                    lift.driveServo(.7);

                return notAtSpeed;
            }
        };
    }

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
            svgLogger.log(String.format(Locale.US,"%.3f,%.3f,%.3f,%.3f", t, power, vel, hoodAngle));

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

                return false;
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
