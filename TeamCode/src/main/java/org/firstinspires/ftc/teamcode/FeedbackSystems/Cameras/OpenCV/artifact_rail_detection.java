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
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class artifact_rail_detection extends OpenCvPipeline
{
   Telemetry telemetry;

   // cam_placement scalar format Scalar(camera height[cm], minimum distance visible from cam[cm], cam-to-arm x_distance[cm], cam-to-arm hinge z_distance[cm])
   final Scalar cam_placement = new Scalar(8*2.54, 12.5, 8.5, 25);
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
   private Scalar object_size_limits = new Scalar(150, 20000);



   // contours, ellipses, rectangles, center dots&bounding box
   public Scalar draw_objects = new Scalar(1, 1, 1, 1);

   final Scalar purple_1_upper = new Scalar(179, 255, 255);
   final Scalar purple_1_lower = new Scalar(135, 35, 100);

//   public Scalar purple_1_lower = new Scalar(143, 51, 93);

//   public Scalar purple_2_upper = new Scalar(10, 255, 255);
//   public Scalar purple_2_lower = new Scalar(10, 50, 50);


   final Scalar green_upper = new Scalar(50, 255, 240);
   final Scalar green_lower = new Scalar(25, 30, 100);


   // must be any odd number > 0
   public int blur = 3;

   // edge contour threshold
   public int threshold = 500;


   private Mat output = new Mat();
   private Mat purple_binary_mat = new Mat();
   private Mat green_binary_mat = new Mat();
   private Mat hsv_mask = new Mat();
   private Mat binary_mask_mat = new Mat();
   private Mat grey = new Mat();
   private Mat drawings = new Mat();
   private ArrayList<Point> artifact_points = new ArrayList<>();
   // 1 is red, -1 is blue
   public double side = 1;
   public double width_to_object_ratio = 24;

   double slope = -Math.tan(2.88);

   double bottom_y_intercept;
   double top_y_intercept;
   Scalar detection_limits;


   public void team_side(double side)
   {
      if (side == 1)
      {
         // red
         this.bottom_y_intercept = 40;
         this.top_y_intercept = 0;
         // x_max, x_min, y_max, y_min
         this.detection_limits = new Scalar(70, 280, 48, 115);
      }

      if (side == -1)
      {
         this.bottom_y_intercept = 55;
         this.top_y_intercept = 30;
         this.detection_limits = new Scalar(70, 280, 48, 115);
      }
   }


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

   public double sloped_line_function(double y_intercept, double slope, double input_x)
   {
      return slope*input_x + y_intercept;
   }


   @Override
   public void init(Mat firstFrame)
   {

   }

   public artifact_rail_detection(Telemetry telemetry) {
      this.telemetry = telemetry;
   }
   @Override
   public Mat processFrame(Mat input)
   {
      team_side(side);
      Scalar white_color = new Scalar(256, 256, 256);
      Scalar blue_color = new Scalar(0, 15, 137); // phthalo blue
      Scalar red_color = new Scalar(255, 0, 0);
      Scalar yellow_color = new Scalar(255, 255, 0);
      Scalar green_color = new Scalar(0, 255, 0);
      ArrayList<Point> artifact_points = new ArrayList<>();
      double diagonal_fov = 55;
      double x_resolution = 320;
      double y_resolution = 180;
      double diagonal_resolution = Math.sqrt(Math.pow(x_resolution, 2) + Math.pow(y_resolution, 2));
      // number of degrees per pixel
      double pixel_angle = diagonal_fov/diagonal_resolution;
      //original size of object
      double original_size = 3.5;
      double x_center = x_resolution/2;

      double leftmost_object_point = x_resolution;
      double rightmost_object_point = 0;
//      telemetry.addData("pixel angle", pixel_angle);


      if (side == -1)
      {
         Core.flip(input, input, 1);
      }


      Imgproc.medianBlur(input, drawings, blur);
      Imgproc.cvtColor(drawings, hsv_mask, Imgproc.COLOR_BGR2HSV);


      // Color mat. Returns a binary (black/white) matrix.
      Core.inRange(hsv_mask, purple_1_lower, purple_1_upper, purple_binary_mat);
      Core.inRange(hsv_mask, green_lower, green_upper, green_binary_mat);
      binary_mask_mat.release();


      // overlay binary matrix onto it onto the input
      Core.bitwise_or(drawings, drawings, binary_mask_mat, green_binary_mat);
      Core.bitwise_or(drawings, drawings, binary_mask_mat, purple_binary_mat);


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
         // Left Bound
         Imgproc.line(drawings, new Point(detection_limits.val[0], sloped_line_function(bottom_y_intercept, slope, detection_limits.val[0])), new Point(detection_limits.val[0], sloped_line_function(top_y_intercept, slope, detection_limits.val[0])), green_color);
         // Right Bound
         Imgproc.line(drawings, new Point(detection_limits.val[1], sloped_line_function(bottom_y_intercept, slope, detection_limits.val[1])), new Point(detection_limits.val[1], sloped_line_function(top_y_intercept, slope, detection_limits.val[1])), green_color);


         // Lower diagonal line
         Imgproc.line(drawings, new Point(detection_limits.val[0], sloped_line_function(top_y_intercept, slope, detection_limits.val[0])), new Point(detection_limits.val[1], sloped_line_function(top_y_intercept, slope, detection_limits.val[1])), green_color);
         // upper diagonal line
         Imgproc.line(drawings, new Point(detection_limits.val[0], sloped_line_function(bottom_y_intercept, slope, detection_limits.val[0])), new Point(detection_limits.val[1], sloped_line_function(bottom_y_intercept, slope, detection_limits.val[1])), green_color);


         // Bottom left to bottom right
//         Imgproc.line(drawings, new Point(detection_limits.val[0], detection_limits.val[2]), new Point(detection_limits.val[1], detection_limits.val[2]), green_color);
         // top right to top left
//         Imgproc.line(drawings, new Point(detection_limits.val[1], detection_limits.val[3]), new Point(detection_limits.val[0], detection_limits.val[3]), green_color);

      }

      // Draw contours, elipses, and rectangles
      for (int i = 0; i < contours.size(); i++) {
         Point object_center_point = minEllipse[i].center;
         // check to see if object is within the set margins
//         if ((object_center_point.x >= detection_limits.val[0] && object_center_point.x <= detection_limits.val[1]) && (object_center_point.y >= detection_limits.val[2] && object_center_point.y <= detection_limits.val[3]))
         if ((object_center_point.x >= detection_limits.val[0] && object_center_point.x <= detection_limits.val[1]) && ((object_center_point.y >= sloped_line_function(top_y_intercept, slope, object_center_point.x)) && (object_center_point.y <= sloped_line_function(bottom_y_intercept, slope, object_center_point.x))))
         {
            if ((minEllipse[i].boundingRect().area() > object_size_limits.val[0]) && (minEllipse[i].boundingRect().area() < object_size_limits.val[1]))
            {
               telemetry.addData("area", minEllipse[i].boundingRect().area());
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
                     if (rectPoints[j].x > rightmost_object_point)
                     {
                        rightmost_object_point = rectPoints[j].x;
                        if (rectPoints[j].x >= detection_limits.val[1])
                        {
                           rightmost_object_point = detection_limits.val[1];
                        }
                     }

                     if (rectPoints[j].x < leftmost_object_point)
                     {
                        leftmost_object_point = rectPoints[j].x;
                     }
                     Imgproc.line(drawings, rectPoints[j], rectPoints[(j + 1) % 4], red_color);
                  }
               }

               // Draw center points
               if (draw_objects.val[3] >= 1)
               {
                  Imgproc.circle(drawings, object_center_point, 1, yellow_color, 2, 8, 0);
               }
               artifact_points.add(object_center_point);
//               telemetry.addData("Dimensions", get_bounding_box_dimensions(rectPoints));
            }
         }
      }
      Imgproc.line(drawings, new Point(leftmost_object_point, 0), new Point(leftmost_object_point, y_resolution), red_color);
      Imgproc.line(drawings, new Point(rightmost_object_point, 0), new Point(rightmost_object_point, y_resolution), red_color);
      double number_of_objects = Math.round((rightmost_object_point-leftmost_object_point)/width_to_object_ratio);
      this.artifact_points = artifact_points;
      telemetry.addData("Artifact Points", artifact_points);
      telemetry.addData("left", leftmost_object_point);
      telemetry.addData("right", rightmost_object_point);
      telemetry.addData("difference", rightmost_object_point-leftmost_object_point);
      telemetry.addData("num balls", number_of_objects);
      telemetry.update();
//      return input;
//      return hsv_mask;
//      return grey;
//      return binary_mask_mat;
//      return canny_output;
      return drawings;
//      Imgproc.cv tColor(input, gray, Imgproc.COLOR_BGR2HSV);



   }
}