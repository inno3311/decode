package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeHW
{

   String m_name;

   public OpMode m_OpMode = null;


   private DcMotor m_intake;


   public IntakeHW(OpMode opMode, String name)
   {
      //super(opMode);
      this.m_name = name;
      this.m_OpMode = opMode;
      init();
   }

   public void init()
   {
      m_OpMode.telemetry.log().add("Adding: ", m_name);
      m_intake = m_OpMode.hardwareMap.get(DcMotorEx.class, m_name);

      m_intake.setDirection(DcMotorSimple.Direction.FORWARD);
   }

   public void intake()
   {
      m_intake.setPower(1);
   }

   public void outtake()
   {
      m_intake.setPower(0);
   }

   public void stop()
   {
      m_intake.setPower(0);
   }

}
