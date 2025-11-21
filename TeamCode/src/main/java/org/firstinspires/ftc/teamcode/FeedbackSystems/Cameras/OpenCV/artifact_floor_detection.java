package org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Collections;

import java.lang.Math;

public class artifact_floor_detection extends OpenCvPipeline
{
   Telemetry telemetry;
   public boolean object_found = false;
   public boolean telemetry_on = false;


   // All measurements are in cm and with the intake center being the origin. {x_dist, y_dist, z_dist, camera pitch, camera yaw}
   final double[] cam_placement = new double[] {16, 14.5, 3.5, 0, 0};
   //public Scalar cam_placement = new Scalar(8, 5.75, 3.7, 2.5);
   double camera_x_offset = cam_placement[1];
   double camera_y_offset = cam_placement[2];
   double camera_z_offset = cam_placement[3];

   // Color_search determines which ball color to search for. 0 is both, 1 is purple, 2 is green
   double color_search = 0;
   double x_resolution = 320;
   double y_resolution = 180;

//   double x_resolution = 640;
//   double y_resolution = 480;

   double y_fov = 52.2;
   double x_fov = 82.1;
   double object_radius = 12.7;

   double x_degrees_per_pixel = x_fov/x_resolution;
   double y_degrees_per_pixel = y_fov/y_resolution;


   private Scalar object_size_limits = new Scalar(80, 10000);
//   private Scalar object_size_limits = new Scalar(20000, 200000000);

   // x_max, x_min, y_max, y_min
   private Scalar detection_limits = new Scalar(0, x_resolution, y_resolution*0.55, y_resolution);

   // contours, ellipses, rectangles, center dots&bounding box
   public Scalar draw_objects = new Scalar(0, 0, 0, 1);

   public Scalar purple_1_upper = new Scalar(179, 255, 255);
   public Scalar purple_1_lower = new Scalar(135, 35, 100);

//   public Scalar purple_1_lower = new Scalar(143, 51, 93);

//   public Scalar purple_2_upper = new Scalar(10, 255, 255);
//   public Scalar purple_2_lower = new Scalar(10, 50, 50);


   public Scalar green_upper = new Scalar(70, 255, 255);
   public Scalar green_lower = new Scalar(32, 35, 32);


   // must be any odd number > 0
   public int blur = 3;

   // edge contour threshold
   public int threshold = 500;

   // Used to help remove/determine if a detection is too wide/thin
   public double width_error_threshold = 0.4;

   private Mat output = new Mat();
   private Mat purple_binary_mat = new Mat();
   private Mat green_binary_mat = new Mat();
   private Mat hsv_mask = new Mat();
   private Mat binary_mask_mat = new Mat();
   private Mat grey = new Mat();
   private Mat drawings = new Mat();
   public double counter = 0;
   private ArrayList<Point> artifact_points = new ArrayList<>();
   private ArrayList<Double> artifact_radii = new ArrayList<>();
   private ArrayList<Point> relative_artifact_locations = new ArrayList<>();


   public ArrayList<Object> get_bounding_box_dimensions(Point[] rectangle_points)
   {
      double a = Math.sqrt(Math.pow((rectangle_points[0].x-rectangle_points[1].x), 2) + Math.pow((rectangle_points[0].y-rectangle_points[1].y), 2));
      double b = Math.sqrt(Math.pow((rectangle_points[1].x-rectangle_points[2].x), 2) + Math.pow((rectangle_points[1].y-rectangle_points[2].y), 2));
      double c = Math.sqrt(Math.pow((rectangle_points[2].x-rectangle_points[3].x), 2) + Math.pow((rectangle_points[2].y-rectangle_points[3].y), 2));
      double d = Math.sqrt(Math.pow((rectangle_points[3].x-rectangle_points[0].x), 2) + Math.pow((rectangle_points[3].y-rectangle_points[0].y), 2));
      ArrayList<Object> rect_dimensions = new ArrayList<>();
      rect_dimensions.add(a);
      rect_dimensions.add(b);
      rect_dimensions.add(c);
      rect_dimensions.add(d);
      return(rect_dimensions);
   }

   public double calculate_artifact_distance(double artifact_radius, double actual_radius, double degrees_per_pixel_x, double offset_scale)
   {
         double angle = Math.toRadians(artifact_radius * degrees_per_pixel_x);
         if (telemetry_on)
         {
         telemetry.addData("angle_degrees", (artifact_radius * degrees_per_pixel_x));
         telemetry.addData("angle_radians", (angle));
         }
         double distance = (actual_radius / Math.tan(angle))*offset_scale;
         return distance;
   }

   public Point calculate_artifact_relative_position(Point position_in_camera, double distance, double degrees_per_pixel_x, double x_resolution, double cam_rel_x, double cam_rel_z)
   {
      double center = x_resolution/2;
      double x_dist_from_center = center - position_in_camera.x;
      double angle = degrees_per_pixel_x * x_dist_from_center;
      double x_distance = distance*Math.sin(-Math.toRadians(angle)) + cam_rel_x;
      double z_distance = distance*Math.cos(Math.toRadians(angle)) + cam_rel_z;
      return new Point(x_distance, z_distance);
   }

   // relative position resources:
   // https://stackoverflow.com/questions/14038002/opencv-how-to-calculate-distance-between-camera-and-object-using-image

   @Override
   public void init(Mat firstFrame)
   {

   }

   public artifact_floor_detection(Telemetry telemetry) {
      this.telemetry = telemetry;
   }
   @Override
   public Mat processFrame(Mat input)
   {
      counter += 1;
      Scalar white_color = new Scalar(256, 256, 256);
      Scalar blue_color = new Scalar(0, 15, 137); // phthalo blue
      Scalar red_color = new Scalar(255, 0, 0);
      Scalar yellow_color = new Scalar(255, 255, 0);
      Scalar green_color = new Scalar(0, 255, 0);
      Scalar purple_color = new Scalar(85, 45, 111);
      ArrayList<Point> artifact_points = new ArrayList<>();
      ArrayList<Double> artifact_radii = new ArrayList<>();
      ArrayList<Point> relative_artifact_locations = new ArrayList<>();

      Imgproc.medianBlur(input, drawings, blur);
      Imgproc.cvtColor(drawings, hsv_mask, Imgproc.COLOR_BGR2HSV);


      // Color mat. Returns a binary (black/white) matrix.
      Core.inRange(hsv_mask, purple_1_lower, purple_1_upper, purple_binary_mat);
      Core.inRange(hsv_mask, green_lower, green_upper, green_binary_mat);
      binary_mask_mat.release();

      if (color_search == 1)
      {
         Core.bitwise_or(drawings, drawings, binary_mask_mat, purple_binary_mat);
      }
      else if (color_search == 2)
      {
         Core.bitwise_or(drawings, drawings, binary_mask_mat, green_binary_mat);
      }
      else
      {
         // overlay binary matrix onto it onto the input
         Core.bitwise_or(drawings, drawings, binary_mask_mat, green_binary_mat);
         Core.bitwise_or(drawings, drawings, binary_mask_mat, purple_binary_mat);
      }


      // Start locating objects
      Imgproc.cvtColor(binary_mask_mat, grey, Imgproc.COLOR_BGR2GRAY);
//      Imgproc.blur(grey, grey, new Size(blur, blur));
      Imgproc.medianBlur(grey, grey, blur);
      Mat canny_output = new Mat();
      Imgproc.Canny(binary_mask_mat, canny_output, threshold, threshold*2);
//      Imgproc.Canny(green_binary_mat, green_canny_output, threshold, threshold*2);

      // add found edges to an array
      List<MatOfPoint> contours = new ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(canny_output, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


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

      // Draw detection bounding box
      if (draw_objects.val[3] >= 1)
      {
         // Bottom left to top left
         Imgproc.line(drawings, new Point(detection_limits.val[0], detection_limits.val[2]), new Point(detection_limits.val[0], detection_limits.val[3]), green_color);
         // Bottom left to bottom right
         Imgproc.line(drawings, new Point(detection_limits.val[0], detection_limits.val[2]), new Point(detection_limits.val[1], detection_limits.val[2]), green_color);
         // top right to bottom right
         Imgproc.line(drawings, new Point(detection_limits.val[1], detection_limits.val[3]), new Point(detection_limits.val[1], detection_limits.val[2]), green_color);
         // top right to top left
         Imgproc.line(drawings, new Point(detection_limits.val[1], detection_limits.val[3]), new Point(detection_limits.val[0], detection_limits.val[3]), green_color);
      }

      // Draw contours, elipses, and rectangles
      object_found = false;
      for (int i = 0; i < contours.size(); i++) {
         Point object_center_point = minEllipse[i].center;
         // check to see if object is within the set margins
         if ((object_center_point.x >= detection_limits.val[0] && object_center_point.x <= detection_limits.val[1]) && (object_center_point.y >= detection_limits.val[2] && object_center_point.y <= detection_limits.val[3]))
         {
            if ((minEllipse[i].boundingRect().area() > object_size_limits.val[0]) && (minEllipse[i].boundingRect().area() < object_size_limits.val[1]))
            {
               if (telemetry_on) {
                  telemetry.addData("area", minEllipse[i].boundingRect().area());
               }
               // Draw contour
               if (draw_objects.val[0] >= 1)
               {
                  Imgproc.drawContours(drawings, contours, i, white_color);
               }

               // Draw rotated ellipse
               if (draw_objects.val[1] >= 1)
               {
                  Imgproc.ellipse(drawings, minEllipse[i], blue_color, 2);
               }

               // Draw rotated rectangle
               Point[] rectPoints = new Point[4];
               minRect[i].points(rectPoints);
               if (draw_objects.val[2] >= 1)
               {
                  for (int j = 0; j < 4; j++)
                  {
                     Imgproc.line(drawings, rectPoints[j], rectPoints[(j + 1) % 4], red_color);
                  }
               }

               //draw attempted_circle
               double average_radius = 0;
               if (draw_objects.val[3] >= 1)
               {
                  Imgproc.line(drawings, new Point(0, y_resolution/2), new Point(x_resolution, y_resolution/2), white_color, (int) y_resolution/180);
                  Imgproc.line(drawings, new Point(x_resolution/2, 0), new Point(x_resolution/2, y_resolution), white_color, (int) y_resolution/180);
                  double width_error = Math.abs(1 - (((double) get_bounding_box_dimensions(rectPoints).get(0))/((double) get_bounding_box_dimensions(rectPoints).get(1))));
                  if (telemetry_on) {
                     telemetry.addData("width_error", width_error);
                  }
                  if (width_error >= width_error_threshold)
                  {
                     continue;
                  }
                  for (int k = 0; k < 4; k++)
                  {
                     average_radius += (double) get_bounding_box_dimensions(rectPoints).get(k) / 2;
                  }
                  average_radius = average_radius / 4;
                  Imgproc.circle(drawings, object_center_point, (int) Math.round(average_radius), white_color, 1);
               }


               // 320x180 resolution
               double artifact_distance = calculate_artifact_distance(average_radius, 6.35, x_degrees_per_pixel, 1.08);

               // live camera (640x480)
               // double artifact_distance = calculate_artifact_distance(artifact_radii, 6.35, x_degrees_per_pixel, 1.8);

               Point relative_position = calculate_artifact_relative_position(object_center_point, artifact_distance, x_degrees_per_pixel, x_resolution, camera_x_offset, camera_z_offset);
               object_found = true;


               if (telemetry_on) {
                  telemetry.addData("Artifact Distance", artifact_distance);
               }
               relative_artifact_locations.add(relative_position);
//               telemetry.addData("Relative Position", relative_position);
               // Draw center points
               if (draw_objects.val[3] >= 1)
               {
                  Imgproc.circle(drawings, object_center_point, 1, yellow_color, 2, 8, 0);
               }
               artifact_points.add(object_center_point);
               artifact_radii.add(average_radius);
               if (telemetry_on) {
                  telemetry.addData("Dimensions", get_bounding_box_dimensions(rectPoints));
               }
            }
         }
      }

      this.artifact_points = artifact_points;
      this.artifact_radii = artifact_radii;
      this.relative_artifact_locations = relative_artifact_locations;
      telemetry.addData("rel pos", relative_artifact_locations);
      telemetry.update();


















//      return input;
//      return hsv_mask;
//      return grey;
//      return canny_output;
//      return binary_mask_mat;
      return drawings;
//      Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2HSV);

   }

   public boolean isObject_found()
   {
      return object_found;
   }

   public ArrayList<Point> getRelative_artifact_locations()
   {
      return relative_artifact_locations;
   }

   public double change_color_search()
   {
      // should cycle values from 0-2
      color_search = (color_search += 1)%3;
      return color_search;
   }

   public double getCounter()
   {
      return counter;
   }

}