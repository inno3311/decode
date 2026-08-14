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

   public enum AllianceColor {
      RED,
      BLUE,
      UNKNOWN
   }

   public enum InitState {
      INIT_START,
      A_B_MENU,
      COLOR_SELECTED
   }

   /////////////////
   /// CLASS MEMBERS
   /////////////////

   //Used to indicate which team color you are on.
   private  StudentTrainingOpMode.AllianceColor m_AllianceColor = StudentTrainingOpMode.AllianceColor.UNKNOWN;

   private InitState m_initState = InitState.INIT_START;


   @Override
   public void init()
   {
      telemetry.log().add("Entering Init");
      telemetry.addData("AC", m_AllianceColor.name());
      if (blackboard.containsKey(BlackboardKeys.ALLIANCE_KEY))
      {
         m_AllianceColor = (AllianceColor) blackboard.get(BlackboardKeys.ALLIANCE_KEY);
         telemetry.log().add("Blackboard loaded ", m_AllianceColor.name());
      } else
      {
         m_AllianceColor = AllianceColor.RED;
      }
      telemetry.addData("AC", m_AllianceColor.name());
      telemetry.log().add("Exiting Init");
   }

   @Override
   public void init_loop()
   {
      switch (m_initState)
      {
         case INIT_START:
            m_initState = InitState.A_B_MENU;
            break;
         case A_B_MENU:
            telemetry.addLine("Select Alliance");
            telemetry.addLine("");
            telemetry.addLine("Press X = BLUE");
            telemetry.addLine("Press B = RED");

            if (gamepad1.bWasPressed())
            {
               m_AllianceColor = AllianceColor.RED;
               telemetry.log().add("Red was selected");
               telemetry.speak("We are RED team");
               m_initState = InitState.COLOR_SELECTED;
            }

            if (gamepad1.xWasPressed())
            {
               m_AllianceColor = AllianceColor.BLUE;
               telemetry.log().add("Blue was selected");
               telemetry.speak("We are BLUE team");
               m_initState = InitState.COLOR_SELECTED;
            }

            break;
         case COLOR_SELECTED:
            telemetry.addLine("Do you wish to start over?");
            if (gamepad1.yWasPressed())
            {
               m_initState = InitState.INIT_START;
            }

            break;
      }
   }

   @Override
   public void loop()
   {

   }

   @Override
   public void stop()
   {
   blackboard.put(BlackboardKeys.ALLIANCE_KEY, m_AllianceColor);
   }
}
