package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.CentricDrive;
import org.firstinspires.ftc.teamcode.Drivebase.Centric.TurnToHeading;
//import org.firstinspires.ftc.teamcode.FeedbackSystems.IMU.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.FeedbackSystems.PID.PIDController;
import org.firstinspires.ftc.teamcode.Drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_floor_detection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.cert.TrustAnchor;

@TeleOp(name = "_Centric Command", group = "FieldCentric")
//@Disabled
public class CentricCommand extends LinearOpMode
{
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMU imu;
    PIDController pid;
    artifact_floor_detection ground_seeker;
    ElapsedTime time;
    double detection_color = 0;

    double color_switch_flag = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCamera();
        ground_seeker = new artifact_floor_detection(telemetry);

        drive = new MecanumDrive(hardwareMap, null);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);

        time = new ElapsedTime();
        time.startTime();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            centricDrive.drive(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    imu.getRobotYawPitchRollAngles().getYaw(),
                    //                turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                    gamepad1.right_trigger,
                    -gamepad1.right_stick_x
            );
            if (ground_seeker.isObject_found()) {
                telemetry.addData("objects", ground_seeker.getRelative_artifact_locations());
            } else {
                telemetry.addData("Objects", ground_seeker.isObject_found());
            }

            if (gamepad1.x && color_switch_flag < time.seconds()) {
                detection_color = ground_seeker.change_color_search();
                color_switch_flag = time.seconds() + 0.25;
            }
            telemetry.addData("detection_color", detection_color);
            telemetry.update();
        }
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
}
