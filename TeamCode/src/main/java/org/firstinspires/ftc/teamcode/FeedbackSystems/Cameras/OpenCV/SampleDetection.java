package org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;


public class SampleDetection extends OpenCvPipeline
{
   private Mat srcGray = new Mat();
   private static final int MAX_THRESHOLD = 255;
   private int threshold = 100;
   Telemetry telemetry;


   // cam_placement scalar format Scalar(camera height[cm], minimum distance visible from cam[cm], cam-to-arm x_distance[cm], cam-to-arm hinge z_distance[cm])
   public Scalar cam_placement = new Scalar(8*2.54, 12.5, 8.5, 25);
   //public Scalar cam_placement = new Scalar(8, 5.75, 3.7, 2.5);
   double camera_height = cam_placement.val[0];
   double distance_minimum_camera = cam_placement.val[1];
   double camera_x_offset = cam_placement.val[2];
   double camera_z_offset = cam_placement.val[3];
   double range_limiter = 9999; //larger = farther range detection
   double x_resolution = 320;
   double y_resolution = 180;
   double y_fov = 52.2;
   double x_fov = 82.1;

   double angle_difference = Math.toDegrees(Math.atan(distance_minimum_camera/camera_height));
   double x_degrees_per_pixel = x_fov/x_resolution;
   double y_degrees_per_pixel = y_fov/y_resolution;

   public Scalar lower_red = new Scalar(0, 171, 75);
   public Scalar lower_blue = new Scalar(0, 0, 140);
   public Scalar lower = new Scalar(0, 171, 75);
   public Scalar upper = new Scalar(255, 255, 255);
   public Scalar blur = new Scalar(1, 1, 0, 0);
   private Mat ycrcbMat = new Mat();
   private Mat binaryMat = new Mat();
   private Mat maskedInputMat = new Mat();
   private double y_distance = 0;
   private ArrayList<Point> sample_points = new ArrayList<>();
   Mat gray = new Mat();

   public void reduce_bounding_boxes(Point rectangle_points)
   {

   }

   public double calculate_bounding_box_area(Point[] rectangle_points)
   {
      double a = Math.sqrt(Math.pow((rectangle_points[0].x-rectangle_points[1].x), 2) + Math.pow((rectangle_points[0].y-rectangle_points[1].y), 2));
      double b = Math.sqrt(Math.pow((rectangle_points[1].x-rectangle_points[2].x), 2) + Math.pow((rectangle_points[1].y-rectangle_points[2].y), 2));
      double c = Math.sqrt(Math.pow((rectangle_points[2].x-rectangle_points[3].x), 2) + Math.pow((rectangle_points[2].y-rectangle_points[3].y), 2));
      double d = Math.sqrt(Math.pow((rectangle_points[3].x-rectangle_points[0].x), 2) + Math.pow((rectangle_points[3].y-rectangle_points[0].y), 2));
      return(a*b);
   }

   @Override
   public void init(Mat firstFrame)
   {

   }

   public SampleDetection(Telemetry telemetry)
   {
      this.telemetry = telemetry;
   }
   @Override
   public Mat processFrame(Mat input)
   {
      ArrayList<Point> sample_points = new ArrayList<>();
      // Create mask to remove all background noise (depending on our color rgb color bounds)
      Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
      Core.inRange(ycrcbMat, lower, upper, binaryMat);
      maskedInputMat.release();
      Core.bitwise_and(input, input, maskedInputMat, binaryMat);

      boolean test = false;
      if (test)
      return binaryMat;

      // Start locating objects
      Imgproc.cvtColor(maskedInputMat, gray, Imgproc.COLOR_BGR2GRAY);
      Imgproc.blur(gray, gray, new Size(blur.val[0], blur.val[1]));
      Mat cannyOutput = new Mat();
      Imgproc.Canny(binaryMat, cannyOutput, threshold, threshold*2);

      boolean cannyTest = false;
      if (cannyTest)
         return cannyOutput;

      // add found edges to an array
      List<MatOfPoint> contours = new ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

      // Find the rotated rectangles and ellipses for each contour
      RotatedRect[] minRect = new RotatedRect[contours.size()];
      RotatedRect[] minEllipse = new RotatedRect[contours.size()];
      for (int i = 0; i < contours.size(); i++) {
         minRect[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
         minEllipse[i] = new RotatedRect();
         if (contours.get(i).rows() > 5) {
            minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
         }
      }

      // Draw contours, elipses, and rectangles
      Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
      telemetry.addData("number of contours", contours.size());
      for (int i = 0; i < contours.size(); i++) {

         if ((minEllipse[i].boundingRect().area() > 1000) && (minEllipse[i].boundingRect().area() < 2000))
         {
            Scalar color = new Scalar(256, 256, 256);
            Scalar color2 = new Scalar(111, 111, 256);
            // Draw contour
            Imgproc.drawContours(input, contours, i, color);
            // Draw ellipse
            Imgproc.ellipse(input, minEllipse[i], color, 2);

            Imgproc.putText(input,String.valueOf(i),minEllipse[i].center,1,1,color2,1,1,true);
            // Draw rotated rectangle
            Point[] rectPoints = new Point[4];
            minRect[i].points(rectPoints);
            double size = calculate_bounding_box_area(rectPoints);
            double y_range_limit = y_resolution / range_limiter;
            //0, 0 is in the top left, that's why we want a LARGER y value than the limit, NOT a smaller
            if (minRect[i].center.y <= y_range_limit)
            {
               continue;
            }
            sample_points.add(minEllipse[i].center);
         }
      }
      this.sample_points = sample_points;
      telemetry.update();
      return input;
   }

   public ArrayList<ArrayList<Double>> object_distances()
   {
      ArrayList<ArrayList<Double>> object_points = new ArrayList<>();
      if (sample_points.size() < 1)
      {
         ArrayList<Double> no_object_point = new ArrayList<>(Arrays.asList(-100.0, -100.0, -100.0));
         return new ArrayList<>(Arrays.asList(no_object_point));
      }      int nearest_object_index = 0;
      for (int i = 0; i < sample_points.size(); i++)
      {
         if (sample_points.get(i).y <= sample_points.get(nearest_object_index).y)
         {
            continue;
         }
         nearest_object_index = i;
      }
      Point position_in_camera = sample_points.get(nearest_object_index);
      double position_in_camera_x = position_in_camera.x;
      double position_in_camera_y = y_resolution - position_in_camera.y;
      double angle = Math.toRadians(angle_difference + y_degrees_per_pixel * (y_resolution - position_in_camera.y));
      double z_distance = camera_height * Math.tan(angle);
      double center_line = (x_degrees_per_pixel*x_resolution)/2;
      double x_angle = (x_degrees_per_pixel*position_in_camera_x)-center_line;
      double x_distance = Math.tan(Math.toRadians(x_angle))*z_distance+camera_x_offset;
      ArrayList<Double> distances = new ArrayList<>(Arrays.asList(x_distance, y_distance, z_distance));
      object_points.add(distances);
      return object_points;
   }

// look into errosion to try and remove overlapping contour lines.
}
