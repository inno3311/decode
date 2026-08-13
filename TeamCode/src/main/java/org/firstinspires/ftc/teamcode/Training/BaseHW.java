package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BaseHW
{
   public OpMode m_OpMode = null;

   ////////////////////////////////////////////////////////////////////////////////////////////////
   /// Add all needed hardware devices here.

   //Default Constructor is made private to force usage of the custom constructor.
   private BaseHW(){}


   public BaseHW(OpMode opMode)
   {
      m_OpMode = opMode;
   }

   public void init()
   {}


}
