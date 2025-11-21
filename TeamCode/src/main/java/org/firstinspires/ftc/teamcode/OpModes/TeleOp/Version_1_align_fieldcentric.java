package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Drivebase.DriveController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.AprilTags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_floor_detection;
import org.firstinspires.ftc.teamcode.FeedbackSystems.PID.PIDController;
import org.firstinspires.ftc.teamcode.Misc.FireControl;
import org.firstinspires.ftc.teamcode.OpModes.UItility.ground_align;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Hood;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Intake;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Shooter;
import org.firstinspires.ftc.teamcode.Robot.CommonFeatures.Trigger;
import org.firstinspires.ftc.teamcode.Robot.v1.Transfer;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "ALIGN_Version_1_Centric")
public class Version_1_align_fieldcentric extends LinearOpMode
{
    Intake intake;
    Shooter shooter;
    Hood hood;
    Trigger trigger;
    Transfer transfer;

    DriveController driveController;
    FireControl fireControl;
    ElapsedTime time;
    double flag1 = 0;
    double flag2 = 0;
    double initialTargetVelocity = 12;
    double[] shooterParameters;
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    PIDController pid;
    artifact_floor_detection ground_seeker;
    ground_align ground_align;
    double detection_color = 0;

    double color_switch_flag = 0;
    double align_flag = 0;

    public void runBlocking2(Action action, Gamepad gpad) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

            running = action.run(packet);

            dash.sendTelemetryPacket(packet);
            telemetry.addData("runBlocking2", "in loop");
            telemetry.update();

            if (gpad.left_stick_button && gpad.right_stick_button)
            {
                break;
            }
        }
        telemetry.addData("runBlocking2", "exit loop");
        telemetry.update();
    }

    private void initCamera()
    {
        //https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/camera_initialization_overview.md
        String camera_name = "Webcam 1";
        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, camera_name);
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        ground_seeker = new artifact_floor_detection(telemetry);
        FtcDashboard.getInstance().startCameraStream(camera,0);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(ground_seeker);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Failed","");
            }
        });
    }



    double target_velocity;
    double target_angle;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake = new Intake(this);
        shooter = new Shooter(this);
        hood = new Hood(this);
        trigger = new Trigger(this);
        transfer = new Transfer(this);

        initCamera();
        ground_seeker = new artifact_floor_detection(telemetry);

        drive = new MecanumDrive(hardwareMap, null);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);

        time = new ElapsedTime();
        waitForStart();
        time.startTime();

        while (opModeIsActive())
        {
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            centricDrive.drive(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    imu.getRobotYawPitchRollAngles().getYaw(),
                    //                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                    gamepad1.right_trigger,
                    gamepad1.right_stick_x
            );

            telemetry.addData("positions", ground_seeker.getRelative_artifact_locations());
            if (gamepad1.a && align_flag < time.seconds())
            {
                align_flag = time.seconds() + 0.25;
                ground_align = new ground_align(new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0))));
                Action align = ground_align.align(ground_seeker.getRelative_artifact_locations().get(0));
//                Action align = ground_align.align(new Point(1, 1));
                runBlocking2(align, gamepad1);
            }
            if (gamepad1.x && color_switch_flag < time.seconds()) {
                detection_color = ground_seeker.change_color_search();
                color_switch_flag = time.seconds() + 0.25;
            }

            if (gamepad1.b)
            {
                telemetry.addData("counter", ground_seeker.getCounter());
            }
            telemetry.addData("detection_color", detection_color);
            telemetry.update();
        }

    }
}
