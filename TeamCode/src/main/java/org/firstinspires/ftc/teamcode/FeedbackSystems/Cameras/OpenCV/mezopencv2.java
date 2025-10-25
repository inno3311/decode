


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


public class mezopencv2 extends OpenCvPipeline
{
   Telemetry telemetry;


   public enum SkystonePosition
   {
      LEFT,
      CENTER,
      RIGHT
   }


   public Scalar lower = new Scalar(0, 178, 75);
   public Scalar upper = new Scalar(255, 255, 255);
   public double threshold = 100;
   //public int blur = 0;

   Mat grey = new Mat();



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

   // Volatile since accessed by OpMode thread w/o synchronization
   private volatile SkystonePosition position = SkystonePosition.LEFT;

   /*
    * Working variables
    */
   Mat region1_Cb, region2_Cb, region3_Cb;
   Mat YCrCb = new Mat();
   Mat Cb = new Mat();
   int avg1, avg2, avg3;



   public mezopencv2(Telemetry telemetry) {
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

      /*
       * Submats are a persistent reference to a region of the parent
       * buffer. Any changes to the child affect the parent, and the
       * reverse also holds true.
       */
//      region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//      region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//      region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
   }

   private Mat srcGray = new Mat();
   private static final int MAX_THRESHOLD = 255;
  //rivate int threshold = 100;
  // private Random rng = new Random(12345);


   @Override
   public Mat processFrame(Mat input)
   {
      Mat cannyOutput = new Mat();

      Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

      //Imgproc.cvtColor(input, srcGray, Imgproc.COLOR_BGR2GRAY);

      Core.inRange(ycrcbMat, lower, upper, binaryMat);

      boolean testGray = false;
      if (testGray)
         return binaryMat;

      //Imgproc.blur(srcGray, srcGray, new Size(3, 3));

      Core.bitwise_and(input, input, maskedInputMat, binaryMat);

      boolean maskedTest = false;
      if (maskedTest)
         return maskedInputMat;

      testGray = false;
      if (testGray)
         return srcGray;

      boolean testInput = false;
      if (testInput)
      return input;

      Imgproc.Canny(maskedInputMat, cannyOutput, threshold, threshold * 2);
      List<MatOfPoint> contours = new ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
      RotatedRect[] minRect = new RotatedRect[contours.size()];
      RotatedRect[] minEllipse = new RotatedRect[contours.size()];
      for (int i = 0; i < contours.size(); i++)
      {
         minRect[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
         minEllipse[i] = new RotatedRect();
         if (contours.get(i).rows() > 5)
         {
            minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
         }




      }
      Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
      for (int i = 0; i < contours.size(); i++)
      {
         Scalar color = new Scalar(111, 0, 0);
         // contour
         Imgproc.drawContours(drawing, contours, i, color);
         // ellipse

         color = new Scalar(0, 111, 0);
         //Imgproc.ellipse(drawing, minEllipse[i], color, 1);
         // rotated rectangle
         Point[] rectPoints = new Point[4];
         minRect[i].points(rectPoints);
         for (int j = 0; j < 4; j++)
         {
            color = new Scalar(0, 0, 111);
            Imgproc.line(drawing, rectPoints[j], rectPoints[(j + 1) % 4], color);
         }

       //  Core.bitwise_and(input,drawing,drawing);
         //Core.bitwise_and();
         //Core.
//      telemetry.addData("width:", contours.get(maxValIdx).width());
//      telemetry.addData("hight:", contours.get(maxValIdx).height());
//      telemetry.addData("contours.size() ", contours.size());
//      telemetry.addData("X:", boundRect[maxValIdx].x);
//      telemetry.addData("Y:", boundRect[maxValIdx].y);

         telemetry.update();

        // return input;
      }
      return drawing;
      //return cannyOutput;
   }
}

