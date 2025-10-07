package org.firstinspires.ftc.teamcode.FeedbackSystems.Cameras.OpenCV;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class SampleSeeker extends OpenCvPipeline
{
   Telemetry telemetry;

   public boolean object_found = false;
   public double angle_x = 0;
   public double angle_y = 0;

    private double get_horizontal_fov(double x_resolution, double y_resolution, double diagonal_fov)
    {
        // Source: https://medium.com/insights-on-virtual-reality/converting-diagonal-field-of-view-and-aspect-ratio-to-horizontal-and-vertical-field-of-view-13bcc1d8600c#:~:text=We%20use%20this%20to%20convert%20between%20field-of-view%20space,space%20and%20then%20converted%20back%20into%20FOV%20space.
        double diagonal_aspect = Math.sqrt(Math.pow(x_resolution, 2) + Math.pow(y_resolution, 2));
        double horizontal_FOV = Math.atan(Math.tan(Math.toRadians(diagonal_fov/2)) * (x_resolution/diagonal_aspect)) * 2;
        horizontal_FOV = Math.toDegrees(horizontal_FOV);
        return(horizontal_FOV);
    }

    private double get_vertical_fov(double x_resolution, double y_resolution, double diagonal_fov)
    {
        // Source: https://medium.com/insights-on-virtual-reality/converting-diagonal-field-of-view-and-aspect-ratio-to-horizontal-and-vertical-field-of-view-13bcc1d8600c#:~:text=We%20use%20this%20to%20convert%20between%20field-of-view%20space,space%20and%20then%20converted%20back%20into%20FOV%20space.
        double diagonal_aspect = Math.sqrt(Math.pow(x_resolution, 2) + Math.pow(y_resolution, 2));
        double vertical_FOV = Math.atan(Math.tan(Math.toRadians(diagonal_fov/2)) * (y_resolution/diagonal_aspect)) * 2;
        vertical_FOV = Math.toDegrees(vertical_FOV);
        return(vertical_FOV);
    }

    private double calculate_distance(double x_distance,double y_distance)
    {
        return(Math.sqrt(Math.pow(x_distance, 2) + Math.pow(y_distance, 2)));
    }


   public Scalar lower = new Scalar(0, 146, 153);
   public Scalar upper = new Scalar(255, 255, 255);
   private double threshold;
   private double max_size_threshold = 3000; //pixels
   private double min_size_threshold = 1000; //pixels
   private double x_resolution = 320;
   private double y_resolution = 180;
   private double diagonal_fov = 90;
   private double x_fov = get_horizontal_fov(x_resolution, y_resolution, diagonal_fov);
   private double y_fov = get_vertical_fov(x_resolution, y_resolution, diagonal_fov);
   private double x_degrees_per_pixel = x_fov/x_resolution;
   private double y_degrees_per_pixel = y_fov/y_resolution;
   private double max_pickup_angle = 30;
   private double camera_x_offset = -1.5; // distance in inches camera is FROM claw center
   private double camera_y_offset  = 2.5; // distance in inches camera is FROM claw center
   private double camera_height = 14; //inches
   /*
    * A good practice when typing EOCV pipelines is
    * declaring the Mats you will use here at the top
    * of your pipeline, to reuse the same buffers every
    * time. This removes the need to call mat.release()
    * with every Mat you create on the processFrame method,
    * and therefore, reducing the possibility of getting a
    * memory leak and causing the app to crash due to an
    * "Out of Memory" error.
    */
   private Mat ycrcbMat       = new Mat();
   private Mat binaryMat      = new Mat();
   private Mat maskedInputMat = new Mat();


   /*
    * Working variables
    */
   Mat region1_Cb, region2_Cb, region3_Cb;
   Mat YCrCb = new Mat();
   Mat Cb = new Mat();
   int avg1, avg2, avg3;



   public SampleSeeker(Telemetry telemetry) {
      this.telemetry = telemetry;
   }
   /*
    * This function takes the RGB frame, converts to YCrCb,
    * and extracts the Cb channel to the 'Cb' variable
    */
   void inputToCb(Mat input)
   {
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(YCrCb, Cb, 2);
   }

   @Override
   public void init(Mat firstFrame)
   {
      /*
       * We need to call this in order to make sure the 'Cb'
       * object is initialized, so that the submats we make
       * will still be linked to it on subsequent frames. (If
       * the object were to only be initialized in processFrame,
       * then the submats would become delinked because the backing
       * buffer would be re-allocated the first time a real frame
       * was crunched)
       */
      inputToCb(firstFrame);


   }


   boolean forceRetrunYcrcbMat = false;
   boolean forceReturnBinaryMat = false;
   boolean displayMaskedInputMat = false;
   boolean displayCannyOutput = false;
   boolean displayDrawing = false;

   @Override
   public Mat processFrame(Mat input)
   {
      /*
       * Converts our input mat from RGB to YCrCb.
       * EOCV ALWAYS returns RGB mats, so you'd
       * always convert from RGB to the color
       * space you want to use.
       *
       * Takes our "input" mat as an input, and outputs
       * to a separate Mat buffer "ycrcbMat"
       */
      //Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
      Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2Lab);
      //Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_BGR2HSV);

if (forceRetrunYcrcbMat)
      return ycrcbMat;

      /*
       * This is where our thresholding actually happens.
       * Takes our "ycrcbMat" as input and outputs a "binary"
       * Mat to "binaryMat" of the same size as our input.
       * "Discards" all the pixels outside the bounds specified
       * by the scalars above (and modifiable with EOCV-Sim's
       * live variable tuner.)
       *
       * Binary meaning that we have either a 0 or 255 value
       * for every pixel.
       *
       * 0 represents our pixels that were outside the bounds
       * 255 represents our pixels that are inside the bounds
       */
      Core.inRange(ycrcbMat, lower, upper, binaryMat);

      if (forceReturnBinaryMat)
         return binaryMat;

      /*
       * Release the reusable Mat so that old data doesn't
       * affect the next step in the current processing
       */
      maskedInputMat.release();

      /*
       * Now, with our binary Mat, we perform a "bitwise and"
       * to our input image, meaning that we will perform a mask
       * which will include the pixels from our input Mat which
       * are "255" in our binary Mat (meaning that they're inside
       * the range) and will discard any other pixel outside the
       * range (RGB 0, 0, 0. All discarded pixels will be black)
       */
      Core.bitwise_and(input, input, maskedInputMat, binaryMat);

      if (displayMaskedInputMat)
         return maskedInputMat;


      /*
       * The Mat returned from this method is the
       * one displayed on the viewport.
       *
       * To visualize our threshold, we'll return
       * the "masked input mat" which shows the
       * pixel from the input Mat that were inside
       * the threshold range.
       */
     // org.opencv.core.Size size = Size.;
      // Imgproc.blur(binaryMat, binaryMat, new Size(4,4));

      Mat cannyOutput = new Mat();
      Imgproc.Canny(binaryMat, cannyOutput, threshold, threshold * 2);

      if (displayCannyOutput)
         return cannyOutput;

      List<MatOfPoint> contours = new ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

      MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
      Rect[] boundRect = new Rect[contours.size()];
      Point[] centers = new Point[contours.size()];
      float[][] radius = new float[contours.size()][1];

      for (int i = 0; i < contours.size(); i++) {
         contoursPoly[i] = new MatOfPoint2f();
         Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
         boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
         centers[i] = new Point();
         Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
      }

      Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);

      List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
      for (MatOfPoint2f poly : contoursPoly) {
         contoursPolyList.add(new MatOfPoint(poly.toArray()));
      }

      //contoursPolyList.get(1).

      Scalar color1 = new Scalar(111, 222, 111);
      Scalar cyan = new Scalar(0, 222, 222);
      Scalar red = new Scalar(255, 0, 0);
      Scalar yellow = new Scalar(255, 255, 0);
      Scalar green = new Scalar(0, 255, 0);
/*
      for (int i = 0; i < contours.size(); i++) {
         Imgproc.drawContours(drawing, contoursPolyList, i, color1);
         Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color1, 2);
         Imgproc.circle(drawing, centers[i], (int) radius[i][0], color1, 2);
      }

 */


      RotatedRect[] minRect = new RotatedRect[contours.size()];
      RotatedRect[] minEllipse = new RotatedRect[contours.size()];
      for (int i = 0; i < contours.size(); i++) {
         minRect[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
         minEllipse[i] = new RotatedRect();
         if (contours.get(i).rows() > 5) {
            minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
         }
      }
/*
      drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
      for (int i = 0; i < contours.size(); i++) {
         Scalar color = new Scalar(34, 111, 56);
         // contour
         Imgproc.drawContours(drawing, contours, i, color);
         // ellipse
           Imgproc.ellipse(input, minEllipse[i], color, 2);
         // rotated rectangle
         Point[] rectPoints = new Point[4];
         minRect[i].points(rectPoints);
         for (int j = 0; j < 4; j++)
         {
            Imgproc.line(drawing, rectPoints[j], rectPoints[(j+1) % 4], color);
         }
      }

 */

/*
    double maxVal = 0;
    int maxValIdx = 0;
    for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        double contourArea = Imgproc.contourArea(contours.get(contourIdx));
        if (maxVal < contourArea)
        {
            maxVal = contourArea;
            maxValIdx = contourIdx;
        }
    }
*/
    if (contours.size() == 0)
        {
            return input;
        }
    //Imgproc.drawContours(input, contoursPolyList, maxValIdx, cyan);
    //Imgproc.rectangle(input, boundRect[maxValIdx].tl(), boundRect[maxValIdx].br(), color1, 2);
    double absolute_center_x = x_resolution/2;
    double absolute_center_y = y_resolution/2;
    Point absolute_center_point = new Point(absolute_center_x, absolute_center_y);
    Imgproc.circle(input, absolute_center_point,1, yellow, 6);
    //Imgproc.line(input, new Point(0, absolute_center_y), new Point(x_resolution, absolute_center_y), yellow, 5);
    //Imgproc.line(input, new Point(absolute_center_x, 0), new Point(absolute_center_x, y_resolution), yellow, 5);
    int nearest_point_ID = 0;
    boolean object_found = false;
    double nearest_point_distance = x_resolution*y_resolution; // this is just to ensure that this is the MAXIMUM value possible and so that we can always find something smaller
    for (int j = 0; j < contours.size(); j++)
       {
           double area = minEllipse[j].size.area();
           if (area < min_size_threshold || area > max_size_threshold)
           {
               continue;
           }
           double angle = minEllipse[j].angle;
           if (angle > 90)
           {
               angle = Math.abs(angle-180);
           }
           if (angle >= max_pickup_angle)
           {
               continue;
           }
           double x_distance = absolute_center_x - minEllipse[j].center.x;
           double y_distance = absolute_center_y -minEllipse[j].center.y;
           if (nearest_point_distance > calculate_distance(x_distance, y_distance))
           {
               object_found = true;
               nearest_point_distance = calculate_distance(x_distance, y_distance);
               nearest_point_ID = j;
           }
           telemetry.addData("area", area);
           Imgproc.ellipse(input, minEllipse[j], cyan, 2);
           Imgproc.circle(input, minEllipse[j].center, 1, cyan, 2);

       }
        telemetry.addData("contours", contours.size());
//      telemetry.addData("Center", absolute_center_point);
//      telemetry.addData("nearest_point_distance:", nearest_point_distance);
//      telemetry.addData("Nearest Point", minEllipse[nearest_point_ID].center);
//      telemetry.addData("nearest point ID:", nearest_point_ID);
      Imgproc.circle(input, minEllipse[nearest_point_ID].center, 1, green, 2);
      Imgproc.circle(input, new Point(0, 0), 1, green, 2);
      Imgproc.ellipse(input, minEllipse[nearest_point_ID], green, 2);
//
      double delta_distance_x = (minEllipse[nearest_point_ID].center.x) - absolute_center_x;
      double delta_distance_y = (absolute_center_y - minEllipse[nearest_point_ID].center.y);
      double x_angle = Math.toRadians(delta_distance_x * x_degrees_per_pixel);
      double y_angle = Math.toRadians(delta_distance_y * y_degrees_per_pixel);
//      telemetry.addData("Object found?", object_found);
//      telemetry.addData("xFOV", x_fov);
//      telemetry.addData("yFOV", y_fov);
//      telemetry.addData("deltaX", delta_distance_x);
//      telemetry.addData("deltaY", delta_distance_y);
//      telemetry.addData("angleX", x_angle);
//      telemetry.addData("angleY", y_angle);
      this.camera_y_offset = camera_y_offset;
      this.camera_x_offset = camera_x_offset;
      this.object_found = object_found;
      this.angle_x = x_angle;
      this.angle_y = y_angle;
   /*
      telemetry.addData("angle: ",minEllipse[maxValIdx].angle);
      telemetry.addData("center x: ",minEllipse[maxValIdx].center.x);
      telemetry.addData("center y: ",minEllipse[maxValIdx].center.y);
      telemetry.addData("hight: ",minEllipse[maxValIdx].size.height);
      telemetry.addData("width: ",minEllipse[maxValIdx].size.width);
      */
//       telemetry.update();


      return input;
   }

    public double getAngle_x()
   {
        return angle_x;
   }
    public double getAngle_y()
    {
        return angle_y;
    }
    public boolean isObject_detected()
    {
        return object_found;
    }

    public double getCamera_height()
    {
        return camera_height;
    }

}