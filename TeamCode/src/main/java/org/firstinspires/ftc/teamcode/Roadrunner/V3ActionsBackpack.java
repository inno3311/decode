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
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Flipper;
import org.firstinspires.ftc.teamcode.Robot.v3.Intake_sort;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterLeft;
import org.firstinspires.ftc.teamcode.Robot.v3.SorterRight;
import org.firstinspires.ftc.teamcode.Robot.v3.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_rail_detection;

public class V3ActionsBackpack
{
    private static final Logger log = LoggerFactory.getLogger(V3ActionsBackpack.class);
    Shooter shooter;
    Intake intake;
    Flipper lift;
    Hood hood;
    Turret turret;

    //Transfer transfer;
    FireControl fireControl;
    ElapsedTime time;
    // Looking from the BACK TO FRONT, Right is purple, left is green
    SorterLeft sorterGreen; // purple
    SorterRight sorterPurple; // green
    artifact_rail_detection railDetection;

    Intake_sort intakeSort;

    ColorSensor colorSensor;

    enum color {GREEN, PURPLE};

    color[][] matrix = new color[3][9];

    public List<AprilTagDetection> taglist;


    double loadTimeStart;
    boolean loadInProgress;

    public int aprilTag_Id = 0;

    private enum FireState {
        INIT,
        SPINUP,
        FIRE,
        RESET_TRIGGER,

        SEC_DELAY,
        TRANSFER_START,
        TRANSFER_STOP,
        RESET,
        DONE
    }

    CsvLogger svgLogger;

    boolean isBlue;

    public V3ActionsBackpack(Shooter shooter, Intake intake, Flipper lift, Hood hood, Turret turret, FireControl fireControl, ElapsedTime time, SorterLeft sorterLeft,
                             SorterRight sorterRight, Intake_sort intakeSort, ColorSensor colorSensor, boolean team)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.lift = lift;
        this.hood = hood;
        this.turret = turret;

        this.isBlue = team;

        //this.railDetection = pipeline;

        //this.transfer = transfer;
        this.fireControl = fireControl;
        this.time = time;
        this.sorterGreen = sorterLeft;
        this.sorterPurple = sorterRight;
        this.intakeSort = intakeSort;
        this.colorSensor = colorSensor;
        time.startTime();

        loadInProgress = false;


        matrix[0][0] = color.GREEN;
        matrix[0][1] = color.PURPLE;
        matrix[0][2] = color.PURPLE;
        matrix[0][3] = color.GREEN;
        matrix[0][4] = color.PURPLE;
        matrix[0][5] = color.PURPLE;
        matrix[0][6] = color.GREEN;
        matrix[0][7] = color.PURPLE;
        matrix[0][8] = color.PURPLE;

        matrix[1][0] = color.PURPLE;
        matrix[1][1] = color.GREEN;
        matrix[1][2] = color.PURPLE;
        matrix[1][3] = color.PURPLE;
        matrix[1][4] = color.GREEN;
        matrix[1][5] = color.PURPLE;
        matrix[1][6] = color.PURPLE;
        matrix[1][7] = color.GREEN;
        matrix[1][8] = color.PURPLE;

        matrix[2][0] = color.PURPLE;
        matrix[2][1] = color.PURPLE;
        matrix[2][2] = color.GREEN;
        matrix[2][3] = color.PURPLE;
        matrix[2][4] = color.PURPLE;
        matrix[2][5] = color.GREEN;
        matrix[2][6] = color.PURPLE;
        matrix[2][7] = color.PURPLE;
        matrix[2][8] = color.GREEN;

//        shooter.setPID(1.0,0.1,0.1,14);
//        PIDFCoefficients cos = shooter.getPID();
//        cos.toString();

//        svgLogger = CsvLogger.getInstance();
//        svgLogger.start("robot_data");
//
//        // Write CSV header
//        svgLogger.log("power,vel,hood angle");
        railDetection = new artifact_rail_detection(fireControl.telemetry);
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

    public Action shootBall(int numRounds, Pose2d pose2d, boolean team, MecanumDrive drive)
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
                Pose2d pose1 = drive.localizer.getPose();
                pose1 = drive.localizer.getPose();
                double target = turret.turretAngleToFixedTarget(pose1.position.x, pose1.position.y, Math.toDegrees(pose1.heading.toDouble()), team, 90);

                switch (state)
                {
                    case INIT:
                        timesFired = 0;

                        shooterParameters = fireControl.firingSuite(pose1, team);
                        shooter.driveToVelocity(shooterParameters[1]);
                        hood.driveToAngleTarget(shooterParameters[0]);
                        state = FireState.SPINUP;

                        packet.put("STATE","INIT");
                        break;
                    case SPINUP:
                        double vel = shooter.getShooter().getVelocity();
                        boolean notAtSpeed = Math.abs(vel - shooterParameters[1]) > 50;

                        if (!notAtSpeed)
                        {
                            state = FireState.FIRE;
                        }

                        packet.put("STATE","SPINUP");
                        packet.put("Target vel", shooterParameters[1]);
                        packet.put("vel",vel);
                        packet.put("notAtSpeed", notAtSpeed);
                        break;
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

                        packet.put("STATE","FIRE_DOWN");
                        break;
                    case TRANSFER_START:
                        sorterGreen.driveServo(-1);
                        transTime = time.seconds();
                        state = FireState.TRANSFER_STOP;

                        packet.put("STATE","TRANSFER");
                        break;
                    case TRANSFER_STOP:
                        if (time.seconds() - transTime > 1)
                        {
                            sorterGreen.driveServo(0);
                            if (timesFired == numRounds)
                            {
                                state = FireState.DONE;
                            }
                            else
                            {
                                state = FireState.SPINUP;
                            }
                        }

                        packet.put("STATE","TRANSFER_STOP");
                        break;
                    case DONE:
                        {
                            shooter.driveToVelocity(0);
                            shooter.getShooter().setPower(0);

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
                sorterPurple.driveServo(speed);
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

    public Action readRamp()
    {
        return new Action()
        {
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                double numBall = railDetection.getNumBalls();
                telemetryPacket.put("NUM BALLS: ", numBall);
                return true;
            }

        };
    }

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
            order = new ArrayList<>(Arrays.asList("purple", "purple", "green"));
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
                    sorterGreen.driveServo(-1);
                }
                else
                // purple
                {
                    sorterPurple.driveServo(1);
                }
                return true;
            }
        };
    }
    public Action read_obelisk(List<AprilTagDetection> currentDetections)
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {

//                boolean found = false;
//                int searchCount = 0;
//                while (!found && searchCount < 5000)
//                {
//                    taglist = fireControl.localizer.getCurrentDetections();
//                    if (!taglist.isEmpty())
//                    {
//                        AprilTagDetection targetTag = null;
//
//                        for (AprilTagDetection detection : taglist)
//                        {
//                            int id = detection.id;
//
//                            if (id >= 21 && id <= 23)
//                            {
//                                targetTag = detection;
//                                found = true;
//                                fireControl.telemetry.addData("id", id);
//                                fireControl.telemetry.update();
//                                break; // stop once found
//                            }
//                        }
//                    }
//                    else
//                    {
//                        searchCount++;
//                        fireControl.telemetry.addData("searchCount", searchCount);
//                        fireControl.telemetry.update();
//                    }
//                }

                Packet.put("current detections", taglist);

                for (AprilTagDetection detection: taglist)
                {
                    if (detection.id == 21)
                    {
                        //sorterRight.driveServo(1);
                        aprilTag_Id = 21;
                        break;
                    }
                    else if (detection.id == 22)
                    {
                        //sorterLeft.driveServo(1);
                        aprilTag_Id = 22;
                        break;
                    }
                    else if (detection.id == 23)
                    {
                        //sorterRight.driveServo(1);
                        //sorterLeft.driveServo(1);
                        aprilTag_Id = 23;
                        break;
                    }
                    else
                    {
                        aprilTag_Id = 21;
                        continue;
                    }
                }

                return true;
            }
        };
    }

    public Action shootBallManual(double velocity, int numRounds, double setvel, double angle,MecanumDrive drive)
    {

        return new Action()
        {
            private boolean initialized = false;
            //private double[] shooterParameters;
            double fireTime;
            double transTime;
            double currentTime;
            int timesFired;

            int railBalls = 0;
            FireState state = FireState.INIT;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                Pose2d pose1 = drive.localizer.getPose();
                pose1 = drive.localizer.getPose();
                double target = turret.turretAngleToFixedTarget(pose1.position.x, pose1.position.y, Math.toDegrees(pose1.heading.toDouble()), isBlue, 90);

                color c;



                switch (state)
                {
                    case INIT:
                        timesFired = 0;
                        railBalls = 0;
                        //shooterParameters = fireControl.firingSuite(velocity, pose1, team);
                        shooter.driveToVelocity(setvel);
                        hood.driveToAngleTarget(angle);
                        state = FireState.TRANSFER_START;

                        //double railballs = railDetection.getNumBalls();
                        //packet.put("railDetection", railballs);

//                        c = matrix[aprilTag_Id-21][timesFired];
//                        if (c == color.PURPLE)
//                        {
//                            sorterLeft.driveServo(-1);
//                        }
//                        else
//                        {
//                            sorterRight.driveServo(1);
//                        }
                        packet.put("MOTIF", aprilTag_Id);
                        packet.put("STATE","INIT");
                        break;
                    case SPINUP:
                        double vel = shooter.getShooter().getVelocity();
                        //boolean notAtSpeed = Math.abs(vel - setvel) > 50;
                        boolean notAtSpeed = (setvel - vel) > 0 ;  //Math.abs(vel - setvel) > 50;

                        if (!notAtSpeed)
                        {
                            state = FireState.FIRE;
                        }

                        packet.put("STATE","SPINUP");
                        packet.put("Target vel", setvel);
                        packet.put("vel",vel);
                        packet.put("notAtSpeed", notAtSpeed);
                        break;
                    case FIRE:
                        fireTime = time.seconds();
                        lift.driveServo(1);
                        state = FireState.RESET_TRIGGER;

                        packet.put("STATE","FIRE");
                        break;
                    case RESET_TRIGGER:
                        currentTime = time.seconds();
                        if (currentTime - fireTime > .3)
                        {
                            lift.driveServo(0);
                            state = FireState.SEC_DELAY;
                            timesFired++;
                        }
                    case SEC_DELAY:
                    {
                        currentTime = time.seconds();
                        if (currentTime - fireTime > 1)
                        {
                            state = FireState.TRANSFER_START;
                        }
                    }
                        packet.put("STATE","FIRE_DOWN");
                        break;
                    case TRANSFER_START:
                        if (timesFired >= numRounds)
                        {
                            state = FireState.DONE;
                        }
                        else
                        {
                            if (aprilTag_Id == 0)
                                aprilTag_Id = 21;
                            c = matrix[aprilTag_Id - 21][timesFired+railBalls];
                            if (timesFired == 2)
                            {
                                sorterPurple.driveServo(1);
                                sorterGreen.driveServo(-1);
                            }
                            else if (c == color.PURPLE)
                            {
                                sorterPurple.driveServo(1);
                            }
                            else
                            {

                                sorterGreen.driveServo(-1);
                            }

                            transTime = time.seconds();
                            state = FireState.TRANSFER_STOP;
                        }

                        packet.put("STATE","TRANSFER");
                        break;
                    case TRANSFER_STOP:
                        if (time.seconds() - transTime > 1)
                        {
                            sorterGreen.driveServo(0);
                            sorterPurple.driveServo(0);
                            if (timesFired > numRounds)
                            {
                                state = FireState.DONE;
                            }
                            else
                            {
                                state = FireState.SPINUP;
                            }
                        }

                        packet.put("STATE","TRANSFER_STOP");
                        break;
                    case DONE:
                    {
                        shooter.driveToVelocity(0);
                        shooter.getShooter().setPower(0);

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

}

