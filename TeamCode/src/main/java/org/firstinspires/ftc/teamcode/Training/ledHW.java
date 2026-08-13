package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ledHW //extends BaseHW
{
   // PWM positions for the RGB Indicator Light
   // These should be adjusted to match the GoBILDA LED's specifications.
   private static final double OFF   = 0.00;
   private static final double RED   = 0.280;
   private static final double GREEN = 0.500;
   private static final double BLUE  = 0.611;


   private Servo m_led;

   String m_name;

   public OpMode m_OpMode = null;


   public ledHW(OpMode opMode, String name)
   {
      //super(opMode);
      this.m_name = name;
      this.m_OpMode = opMode;
      init();
   }

   public void init()
   {
      m_OpMode.telemetry.log().add("Adding: ", m_name);
      m_led = m_OpMode.hardwareMap.get(Servo.class, m_name);
   }

   public void off() {
      m_led.setPosition(OFF);
   }

   public void red() {
      m_led.setPosition(RED);
   }

   public void green() {
      m_led.setPosition(GREEN);
   }

   public void blue() {
      m_led.setPosition(BLUE);
   }

}

