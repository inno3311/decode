package org.firstinspires.ftc.teamcode.Outreach.Nessie.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.IMU.IMUControl;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.fieldCentric.CentricDrive;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.fieldCentric.TurnToHeading;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.initialization.Initialization;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@TeleOp(name = "TeleOp", group = "proto")
public class NessieTeleOp extends LinearOpMode
{
    // Sensors
    IMUControl imu;
    TouchSensor slideLimit;
    TouchSensor elbowLimit;
    Initialization initialization;

    // DriveBase
    MecanumDrive drive;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;

    // Accessories
    Slide slide;
    Elbow elbow;
    Hang hang;
    Wrist wrist;
    Claw claw;

    //Other
    ElapsedTime time;
    private double hangFlag = 0;

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

    @Override
    public void runOpMode() throws InterruptedException
    {
        imu = new IMUControl(hardwareMap, telemetry);
        slideLimit = hardwareMap.get(TouchSensor.class, "slideLimit");
        elbowLimit = hardwareMap.get(TouchSensor.class, "elbowLimit");

        drive = new MecanumDrive(hardwareMap, null);
        turnToHeading = new TurnToHeading(telemetry, drive, imu);
        centricDrive = new CentricDrive(drive, telemetry);

        slide = new Slide(this);
        elbow = new Elbow(this);
        hang = new Hang(this);
        wrist = new Wrist(this);
        claw = new Claw(this);

        time = new ElapsedTime();
        time.startTime();

        //initCamera();

        if (new File("/sdcard/FIRST/blocks/sounds/second.wav").exists())
        {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, new File("/sdcard/FIRST/blocks/sounds/second.wav"));
        }

        waitForStart();

        // DriveBase
        // ==========================================================================================================================================================================

        while (opModeIsActive())
        {
            // Drive Code
            centricDrive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, imu.getAngle(), gamepad1.right_trigger,
                    centricDrive.whichTurnMode(turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.2, 0.2),
                            gamepad1.right_stick_x, gamepad1.back, time.seconds())
            );
//            drive.gamepadController(gamepad1);

            if (gamepad1.y)
            {
                imu.resetAngle();
            }




            // Slide and Elbow
            // ==========================================================================================================================================================================

            boolean runSlide = false;
            if (slide.getMotorPosition() <= -1050) // The Greater then half is where the limit will kick in
            {
                runSlide = true;
                slide.encoderControl(-1000, 0.5);
            }

            if (slide.getMotorPosition() <= -10)// The Greater then half is where the limit will kick in
            {
                runSlide = true;
                slide.encoderControl(0, 0.5);
            }

            if (hangFlag > time.seconds()) {}
            else if (gamepad2.dpad_up)
            {
                slide.encoderPresets(Slide.Presets.TOP_CHAMBER);
                elbow.encoderPresets(Elbow.Presets.TOP_CHAMBER);
            }
            else if (gamepad2.dpad_down)
            {
                slide.encoderPresets(Slide.Presets.PICKUP_WALL);
                elbow.encoderPresets(Elbow.Presets.PICKUP_WALL);
                wrist.driveServo(0.7);

            }
            else if (gamepad2.dpad_left)
            {
                slide.encoderPresets(Slide.Presets.TOP_BUCKET);
                elbow.encoderPresets(Elbow.Presets.TOP_BUCKET);
            }
            else if (gamepad2.dpad_right)
            {

            }
            else if (runSlide)
            {
                elbow.analogControl(1, gamepad2.right_stick_y, false, false, elbowLimit.isPressed(), -3300, true);
            }
            else
            {
                slide.analogControl(1, gamepad2.left_stick_y, true, gamepad2.left_stick_button, slideLimit.isPressed(), -2175, true);
                elbow.analogControl(1, gamepad2.right_stick_y, false, gamepad2.right_stick_button, elbowLimit.isPressed(), -3300, true);
            }

            if (elbowLimit.isPressed())
            {
                elbow.encoderControl(-3,1);
            }




            // Claw and Wrist
            // ==========================================================================================================================================================================

            if (gamepad2.right_bumper) //close
            {
                claw.driveServo(0);
            }
            else if (gamepad2.b && !gamepad2.start) //Half open
            {
                claw.driveServo(0.5);
            }
            else if (gamepad2.right_trigger > 0.2) //open
            {
                claw.driveServo(1);
            }

            slide.automaticEncoderReset(slideLimit.isPressed());
            elbow.automaticEncoderReset(elbowLimit.isPressed());


            // telemetry
            slide.telemetry();
            elbow.telemetry();
            hang.telemetry();
//            telemetry.update();
        }

    }
//    private void initCamera()
//    {
//        //https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/camera_initialization_overview.md
//        String camera_name = "Webcam 2";
//        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, camera_name);
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
//        seeker = new SampleSeeker(telemetry);
//        FtcDashboard.getInstance().startCameraStream(camera,0);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
//                camera.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
//                camera.setPipeline(seeker);
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                telemetry.addData("Camera Failed","");
//            }
//        });
//    }

    private double calculate_camera_height(double arm_angle, double arm_length, double camera_length_offset, double height_offset)
    {
        double height = (Math.sin(Math.toRadians(arm_angle)) * (arm_length + camera_length_offset)) + height_offset;
        return(height);
    }

    private double calculate_x_distance(double x_angle, double camera_height, double camera_x_offset)
    {
        double distance_x = (Math.tan(x_angle)*camera_height) + camera_x_offset;
        return(distance_x);
    }

    private double calculate_y_distance(double y_angle, double camera_height, double camera_y_offset)
    {
        double distance_y = (Math.tan(y_angle)*camera_height) + camera_y_offset;
        return(distance_y);
    }


    private double calculate_x_speed(double x_angle)
    {
        double x_speed = (Math.tan(x_angle));
        return(x_speed);
    }

    private double calculate_y_speed(double y_angle)
    {
        double y_speed = (Math.tan(y_angle));
        return(y_speed);
    }

}
