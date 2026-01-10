package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.ColorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.Misc.CsvLogger;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_rail_detection;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;

public class V3ActionsBackpack
{
    private static final Logger log = LoggerFactory.getLogger(V3ActionsBackpack.class);
    Shooter shooter;
    Intake intake;
    Trigger lift;
    Hood hood;
    Turret turret;

    //Transfer transfer;
    FireControl fireControl;
    ElapsedTime time;
    // Looking from the BACK TO FRONT, Right is purple, left is green
    SorterLeft sorterLeft; // purple
    SorterRight sorterRight; // green
    artifact_rail_detection railDetection;
    AprilTagLocalizer aprilTagLocalizer;

    Intake_sort intakeSort;

    ColorSensor colorSensor;


    double loadTimeStart;
    boolean loadInProgress;



    private enum FireState {
        INIT,
        SPINUP,
        AIM,
        FIRE,
        RESET_TRIGGER,
        TRANSFER_START,
        TRANSFER_STOP,
        RESET,
        DONE
    }

    CsvLogger svgLogger;

    public V3ActionsBackpack(Shooter shooter, Intake intake, Trigger lift, Hood hood, Turret turret, FireControl fireControl, ElapsedTime time, SorterLeft sorterLeft,
    SorterRight sorterRight, Intake_sort intakeSort, ColorSensor colorSensor)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.lift = lift;
        this.hood = hood;
        this.turret = turret;

        //this.transfer = transfer;
        this.fireControl = fireControl;
        this.time = time;
        this.sorterLeft = sorterLeft;
        this.sorterRight = sorterRight;
        this.intakeSort = intakeSort;
        this.colorSensor = colorSensor;
        time.startTime();

        loadInProgress = false;


//        shooter.setPID(1.0,0.1,0.1,14);
//        PIDFCoefficients cos = shooter.getPID();
//        cos.toString();

//        svgLogger = CsvLogger.getInstance();
//        svgLogger.start("robot_data");
//
//        // Write CSV header
//        svgLogger.log("power,vel,hood angle");
    }

    public Action startLogging()
    {
        return new Action()
        {
            public boolean run(@NonNull TelemetryPacket packet)
            {
                packet.put("targetVel", 0);
                packet.put("vel", 0);
                packet.put("power", 0);

                FtcDashboard dashboard = FtcDashboard.getInstance();
                dashboard.sendTelemetryPacket(packet);
                return false;
            };
        };
    }

    public Action shootBall(double velocity, int numRounds, Pose2d pose2d, boolean team, MecanumDrive drive)
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

                        Pose2d pose1 = drive.localizer.getPose();

                        shooterParameters = fireControl.firingSuite(velocity, pose1, team);
                        shooter.driveToVelocity(shooterParameters[1]);
                        hood.driveToAngleTarget(shooterParameters[0]);
                        state = FireState.SPINUP;

                        //packet.put("targetVel",m_targetVelocity);
                        //packet.put("Angle",shooterParameters[0]);
                        packet.put("STATE","INIT");
                        break;
                    case SPINUP:
                        double vel = shooter.getShooter().getVelocity();
                        boolean notAtSpeed = Math.abs(vel - shooterParameters[1]) > 100;

                        if (!notAtSpeed)
                        {
                            state = FireState.AIM;
                        }

//                        packet.put("vel",vel);
                        packet.put("STATE","SPINUP");
                        packet.put("Turret", Math.abs(turret.getError()));
                        packet.put("notAtSpeed", notAtSpeed);
                        break;
                    case AIM:
                         pose1 = drive.localizer.getPose();
                        //turret.trackGoal(pose2d.heading.real, pose2d, team);
                        turret.turretAngleToFixedTarget(pose1.position.x, pose1.position.y, Math.toDegrees(pose1.heading.toDouble()));

                        if (Math.abs(turret.getError()) < 5)
                        {
                            state = FireState.FIRE;
                        }
                    case FIRE:
                        fireTime = time.seconds();
                        lift.driveServo(1);
                        state = FireState.RESET_TRIGGER;
                        packet.put("STATE","FIRE");
                        break;
                    case RESET_TRIGGER:
                        currentTime = time.seconds();
                        if (currentTime - fireTime > .5)
                        {
                            lift.driveServo(0);
                            state = FireState.TRANSFER_START;
                            timesFired++;
                        }
                        //packet.put("fireSeconds",currentTime - fireTime);
                        packet.put("STATE","FIRE_DOWN");
                        break;
                    case TRANSFER_START:
                        sorterLeft.driveServo(-1);
                        transTime = time.seconds();
                        state = FireState.TRANSFER_STOP;
                        packet.put("STATE","TRANSFER");
                        break;
                    case TRANSFER_STOP:
                        if (time.seconds() - transTime > 1)
                        {
                            sorterLeft.driveServo(0);
                            if (timesFired == numRounds)
                            {
                                state = FireState.DONE;
                            }
                            else
                            {
                                state = FireState.AIM;
                            }
                        }
                        packet.put("STATE","TRANSFER_STOP");
                        break;
                    case DONE:
                        {
                            shooter.driveToVelocity(0);
                            shooter.getShooter().setPower(0);
                            //transfer.driveServo(0);
                            //intake.setPower(0);
                            packet.put("STATE","DONE");
                            return false;
                        }
                }


                packet.put("power",shooter.getShooter().getPower());
                packet.put("vel",shooter.getShooter().getVelocity());

                return true;
            }
        };
    }


    public Action mezRampUp(double power)
    {

        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                shooter.getShooter().setPower(power);

                return false;
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

    public Action setHood(double position)
    {
        return new Action()
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {

                hood.driveToAngleTarget(position);



                return false;
            }
        };
    }


    public Action intakeBall(double speed)
    {
        return new Action()
        {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                intake.setPower(speed-0.4);
                intakeSort.setPower(1);

                return false;
            }
        };
    }

    public Action intakeSortBall(double speed)
    {
        return new Action()
        {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                //if (!initialized)
                //{
                intakeSort.setPower(speed);
                //initialized = true;
                //}

                return false;
            }
        };
    }

    public Action sorterRightBall(double speed)
    {
        return new Action()
        {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                //if (!initialized)
                //{
                sorterRight.driveServo(speed);
                //initialized = true;
                //}

                return false;
            }
        };
    }


    public Action intakeColor(double seconds)
    {
        return new Action()
        {
            NormalizedRGBA colors;
            double hue = 0;

            double startTime = 0;

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                if (!initialized)
                {
                    startTime = time.seconds();
                    initialized = true;
                }


                colors = colorSensor.getDetectedColor();
                float normalizedRed, normalizedGreen, normalizedBlue;
                normalizedRed = colors.red / colors.alpha;
                normalizedGreen = colors.green / colors.alpha;
                normalizedBlue = colors.blue / colors.alpha;
                if (JavaUtil.colorToHue(colors.toColor()) != 0)
                {
                    hue = JavaUtil.colorToHue(colors.toColor());
                }
                intake.setPower(1);

                if (hue <= 200) // if GREEN intake to the LEFT side (looking from the back)
                {
                    intake.setPower(-1);
                    intakeSort.setPower(1);
                }
                // if purple (>200) intake to the RIGHT side (looking from the back)
                else
                {
                    intake.setPower(-1);
                    intakeSort.setPower(-1);
                }

                if (time.seconds() > startTime + seconds)
                {
                    intake.setPower(0);
                    intakeSort.setPower(0);
                    return false;
                }

                return true;
            }
        };
    }
//    public Action loadBall(double speed)
//    {
//        return new Action()
//        {
//            //private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket)
//            {
//
//
//                if (!loadInProgress)
//                {
//                    loadInProgress = true;
//                    loadTimeStart = time.seconds();
//                    sorterRight.driveServo(1);
//
//                    return false;
//                }
//
//                if (loadTimeStart - time.seconds() > 1.0)
//                {
//                    sorterRight.driveServo(0);
//                    loadInProgress = false;
//                    return true;
//                }
//
//                return false;
//            }
//        };
//    }

    public Action loadBall(double obelisk_id)
    {
        ArrayList<String> order;
        if (obelisk_id == 21) // GPP
        {
            order = new ArrayList<>(Arrays.asList("green", "purple", "purple"));
        }
        else if (obelisk_id == 22) // PGP
        {
            order = new ArrayList<>(Arrays.asList("purple", "green", "purple"));
        }
        else //ID = 23 PPG
        {
            order = new ArrayList<>(Arrays.asList("purple", "green", "purple"));
        }

        double numBalls = railDetection.getNumBalls();
        String shoot_color = order.get((int) (numBalls%3));
        return new Action()
        {
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                if (Objects.equals(shoot_color, "green"))
                // green
                {
                    sorterLeft.driveServo(-1);
                }
                else
                // purple
                {
                    sorterRight.driveServo(1);
                }
                return true;
            }
        };
    }
    public Action read_obelisk()
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                int id_num = aprilTagLocalizer.getDetectionID();

                if (id_num == 21) // GPP
                {
                    sorterRight.driveServo(1);
                }
                else if (id_num == 22) // PGP
                {
                    sorterLeft.driveServo(1);
                }
                else //ID = 23 PPG
                {
                    sorterRight.driveServo(1);
                    sorterLeft.driveServo(1);
                }
                return true;
            }
        };
    }
}

