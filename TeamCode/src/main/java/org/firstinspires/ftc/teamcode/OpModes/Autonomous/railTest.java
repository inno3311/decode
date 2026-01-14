package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV.artifact_rail_detection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Artifact Rail Test")
public class railTest extends LinearOpMode
{

    OpenCvCamera camera;
    artifact_rail_detection pipeline;

    @Override
    public void runOpMode() {

        // -------------------------------
        // 1. Camera setup
        // -------------------------------
        int cameraMonitorViewId = hardwareMap.appContext
            .getResources()
            .getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
            );

        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"),
            cameraMonitorViewId
        );

        // -------------------------------
        // 2. Create & attach pipeline
        // -------------------------------
        pipeline = new artifact_rail_detection(telemetry);
        camera.setPipeline(pipeline);

        // -------------------------------
        // 3. Open camera async
        // -------------------------------
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // -------------------------------
        // 4. WAIT FOR START
        // -------------------------------
        waitForStart();

        // -------------------------------
        // 5. Use vision results
        // -------------------------------
        while (opModeIsActive()) {

            // Example – depends on YOUR pipeline
            telemetry.addData("Num Balls", pipeline.getNumBalls());


            telemetry.update();
        }

        camera.stopStreaming();
    }
}
