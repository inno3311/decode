package org.firstinspires.ftc.teamcode.OpModes.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Student BioBuzz Proto", group = "OpMode")
public class StudentTrainingOpMode extends OpMode
{
   public static final class BlackboardKeys {

      public static final String ALLIANCE_KEY = "alliance";
      public static final String START_POSITION = "startPosition";

      private BlackboardKeys() {}
   }

   public enum AllianceColor2 {
      RED,
      BLUE,
      UNKNOWN
   }

   public enum InitState2 {
      INIT_START
   }

   /////////////////
   /// CLASS MEMBERS
   /////////////////

   //Used to indicate which team color you are on.
   private  StudentTrainingOpMode.AllianceColor2 m_AllianceColor = StudentTrainingOpMode.AllianceColor2.UNKNOWN;

   private InitState2 m_initState = InitState2.INIT_START;


   @Override
   public void init()
   {
      telemetry.log().add("Entering init");

      telemetry.addData("Alliance: ", m_AllianceColor.name());

      telemetry.log().add("Exiting init");
   }

   @Override
   public void init_loop()
   {

   }

   @Override
   public void loop()
   {

   }

   @Override
   public void stop()
   {

   }
}
