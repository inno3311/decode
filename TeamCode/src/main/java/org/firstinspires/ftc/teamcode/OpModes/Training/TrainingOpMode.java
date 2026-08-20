
package org.firstinspires.ftc.teamcode.OpModes.Training;


import org.firstinspires.ftc.teamcode.Training.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

@TeleOp(name = "BioBuzz Proto", group = "OpMode")
public class TrainingOpMode extends OpMode
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

   private enum InitState {
      INIT_WAIT_FOR_ALLIANCE,
      INIT_SELECT_RED,
      INIT_SELECT_BLUE,
      INIT_READY
   }


   private enum IntakeTestState {
      IDLE,
      INIT,
      TRAVEL,
      STOP
   }

   /////////////////
   /// CLASS MEMBERS
   /////////////////

   //Used to indicate which team color you are on.
   public AllianceColor m_AllianceColor = TrainingOpMode.AllianceColor.UNKNOWN;


   public InitState initState = InitState.INIT_WAIT_FOR_ALLIANCE;

   IntakeTestState m_intakeTestState = IntakeTestState.IDLE;


   private final ElapsedTime loopTimer = new ElapsedTime();
   private double avg = 0;
   private double min = Double.MAX_VALUE;
   private double max = 0;

   private final ElapsedTime m_testTimer = new ElapsedTime();

   ///
   /// Hardware
   ///
   private ledHW m_led;

   private DriveBaseHW m_driveBase;

   private IntakeHW m_intake;

   @Override
   public void init()
   {
      telemetry.log().add("Entering init");

      m_driveBase = new DriveBaseHW();
      m_driveBase.init(this.hardwareMap);

      m_intake = new IntakeHW(this, "intake");

      m_led = new ledHW(this, "led");
      m_led.green();
      if (m_AllianceColor != null)
         telemetry.log().add(m_AllianceColor.name());

      //fetch the stored alliance color.  (In this case it will be unknown)
      if (blackboard.containsKey(BlackboardKeys.ALLIANCE_KEY))
      {
         m_AllianceColor =
               (AllianceColor) blackboard.get(BlackboardKeys.ALLIANCE_KEY);
      }

      if (m_AllianceColor != null)
         telemetry.log().add(m_AllianceColor.name());

      telemetry.log().add("Exiting init");
   }

   @Override
   public void init_loop()
   {
      switch (initState) {

         case INIT_WAIT_FOR_ALLIANCE:

            telemetry.addLine("Select Alliance");
            telemetry.addLine("");
            telemetry.addLine("Press X = BLUE");
            telemetry.addLine("Press B = RED");

            if (gamepad1.xWasPressed())
            {
               m_AllianceColor = AllianceColor.BLUE;
               blackboard.put(BlackboardKeys.ALLIANCE_KEY,m_AllianceColor);
               initState = InitState.INIT_SELECT_BLUE;
               m_led.blue();
            }
            else if (gamepad1.bWasPressed())
            {
               m_AllianceColor = AllianceColor.RED;
               blackboard.put(BlackboardKeys.ALLIANCE_KEY,m_AllianceColor);
               initState = InitState.INIT_SELECT_RED;
               m_led.red();
               //telemetry.addData("Alliance", initState);
            }
            break;
         case INIT_SELECT_BLUE:
            telemetry.log().add("BLUE WAS SELECTED!");
            telemetry.speak("BLUE is selected.");
            initState = InitState.INIT_READY;
            break;
         case INIT_SELECT_RED:
            telemetry.log().add("RED WAS SELECTED!");
            telemetry.speak("RED is selected.");
            initState = InitState.INIT_READY;
            break;
         case INIT_READY:

            telemetry.addLine("Robot Ready!");
            telemetry.addLine("Press Y to reselect color.");

            //telemetry.addData("Alliance", blackboard.alliance);

            telemetry.addLine("");
            telemetry.addLine("Press START to begin.");

            if (gamepad1.yWasPressed())
            {
               initState = InitState.INIT_WAIT_FOR_ALLIANCE;
            }
            break;
      }
   }

   @Override
   public void start()
   {
      avg = 0;
      min = Double.MAX_VALUE;
      max = 0;
      loopTimer.reset();
   }

   @Override
   public void loop()
   {

      m_driveBase.drive(-gamepad1.right_stick_y,gamepad1.right_stick_x,-gamepad1.left_stick_x);

      //  User input from gamepad1.
      //  Action depends on current test state.
      //  Will either start or stop the test.
      if (gamepad1.yWasPressed())
      {
         if (Objects.requireNonNull(m_intakeTestState) == IntakeTestState.IDLE)
         {
            m_intakeTestState = IntakeTestState.INIT;
         }
         else
         {
            m_intakeTestState = IntakeTestState.STOP;
         }
      }

      /// Robot Control Code

      switch (m_intakeTestState)
      {
         case IDLE:
            // do nothing
         break;
         case INIT:
            m_intake.intake();
            m_driveBase.drive(.5,0,0);
            m_testTimer.reset();
            m_intakeTestState = IntakeTestState.TRAVEL;
            break;
         case TRAVEL:
            if (m_testTimer.seconds() > 3)
            {
               m_intakeTestState = IntakeTestState.STOP;
            }
            break;
         case STOP:
            m_intake.stop();
            m_driveBase.drive(0,0,0);
            m_intakeTestState = IntakeTestState.IDLE;
            break;
      }


      ///  Telemetry loop times
      double dt = loopTimer.milliseconds();
      loopTimer.reset();

      avg = avg * 0.98 + dt * 0.02;
      min = Math.min(min, dt);
      max = Math.max(max, dt);

      telemetry.addData("loopFrequency", 1000.0 / dt);
      telemetry.addData("Loop", "%.2f", dt);
      telemetry.addData("Avg", "%.2f", avg);
      telemetry.addData("Min", "%.2f", min);
      telemetry.addData("Max", "%.2f", max);

   }

   @Override
   public void stop()
   {
      blackboard.put(BlackboardKeys.ALLIANCE_KEY,m_AllianceColor);
      super.stop();
   }
}
