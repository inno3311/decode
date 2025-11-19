package org.firstinspires.ftc.teamcode.OpModes.UItility;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Camera Calibration", group = "Vision")
public class CameraCalibrationOpMode extends LinearOpMode {

   private OpenCvCamera camera;
   private CalibrationPipeline pipeline;

   @Override
   public void runOpMode() throws InterruptedException {
      WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
      camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

      pipeline = new CalibrationPipeline(9, 6, telemetry);
      camera.setPipeline(pipeline);

      telemetry.addLine("Opening camera...");
      telemetry.update();

      camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
         public void onOpened() {
            camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            telemetry.addLine("Camera ready. Aim at chessboard.");
            telemetry.update();
         }

         @Override
         public void onError(int errorCode) {
            telemetry.addData("Camera failed to open", errorCode);
            telemetry.update();
         }
      });

      waitForStart();

      while (opModeIsActive()) {
         if (gamepad1.a) {
            pipeline.captureFrame();
            sleep(500);
         }
         if (gamepad1.b) {
            pipeline.computeCalibration();
            sleep(1000);
         }
         telemetry.update();
      }

      camera.stopStreaming();
   }

   public static class CalibrationPipeline extends OpenCvPipeline {
      private int boardWidth, boardHeight;
      private Size frameSize = new Size(1280, 720);
      private Mat gray = new Mat();
      private List<Mat> imagePoints = new ArrayList<>();
      private List<Mat> objectPoints = new ArrayList<>();
      private MatOfPoint3f objp;
      private Telemetry telemetry;

      public CalibrationPipeline(int boardWidth, int boardHeight, Telemetry telemetry) {
         this.boardWidth = boardWidth;
         this.boardHeight = boardHeight;
         this.telemetry = telemetry;

         objp = new MatOfPoint3f();
         for (int i = 0; i < boardHeight; i++) {
            for (int j = 0; j < boardWidth; j++) {
               objp.push_back(new MatOfPoint3f(new Point3(j, i, 0)));
            }
         }
      }

      @Override
      public Mat processFrame(Mat input) {
         Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
         Size patternSize = new Size(boardWidth, boardHeight);
         MatOfPoint2f corners = new MatOfPoint2f();
         boolean found = Calib3d.findChessboardCorners(gray, patternSize, corners);

         if (found) {
            Point[] points = corners.toArray();
            for (Point p : points) {
               Imgproc.circle(input, p, 5, new Scalar(0, 255, 0), 2);
            }
         }

         return input;
      }

      public void captureFrame() {
         telemetry.addLine("Captured frame...");
         objectPoints.add(objp);
         imagePoints.add(gray.clone());
         telemetry.addData("Samples", imagePoints.size());
         telemetry.update();
      }

      public void computeCalibration() {
         telemetry.addLine("Computing calibration...");
         Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
         Mat distCoeffs = Mat.zeros(8, 1, CvType.CV_64F);
         List<Mat> rvecs = new ArrayList<>();
         List<Mat> tvecs = new ArrayList<>();

         double error = Calib3d.calibrateCamera(objectPoints, imagePoints, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs);
         telemetry.addData("Reprojection Error", error);
         telemetry.update();

         saveCalibration(cameraMatrix, distCoeffs);
      }

      private void saveCalibration(Mat cameraMatrix, Mat distCoeffs) {
         String path = "/sdcard/FIRST/camera_calibration.json";
         try (FileWriter writer = new FileWriter(path)) {
            writer.write("{\n");
            writer.write("\"cameraMatrix\": " + matToJson(cameraMatrix) + ",\n");
            writer.write("\"distCoeffs\": " + matToJson(distCoeffs) + "\n");
            writer.write("}\n");
            telemetry.addData("Saved calibration to", path);
         } catch (IOException e) {
            telemetry.addData("Error saving file", e.getMessage());
         }
      }

      private String matToJson(Mat mat) {
         int rows = mat.rows();
         int cols = mat.cols();
         StringBuilder sb = new StringBuilder("[");
         for (int i = 0; i < rows; i++) {
            sb.append("[");
            for (int j = 0; j < cols; j++) {
               sb.append(mat.get(i, j)[0]);
               if (j < cols - 1) sb.append(", ");
            }
            sb.append("]");
            if (i < rows - 1) sb.append(", ");
         }
         sb.append("]");
         return sb.toString();
      }
   }
}

