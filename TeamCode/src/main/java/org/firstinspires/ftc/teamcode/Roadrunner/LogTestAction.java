package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Misc.CsvLogger;

public class LogTestAction implements Action
{
   private long lastTime = 0;
   private final CsvLogger logger = CsvLogger.getInstance();
   private final FtcDashboard dashboard = FtcDashboard.getInstance();

   @Override
   public boolean run(@NonNull TelemetryPacket packet) {
      long now = System.currentTimeMillis();

      for (int i = 0; i < 20; i++)
      {
         if (!logger.isActive())
         {
            packet.put("logger not active:", i);
         }
         //logger.log("Time: " + i);
         //packet.addLine("i = " + i);
         packet.put("i:", i);
         //packet.addLine("Status: Mez Running custom action");
         //dashboard.sendTelemetryPacket(packet);
         //RobotLog.d("SomeUsefulPrefixHere:Some Useful Message Here");
      }

//      if (now - lastTime > 200) {  // throttle logs
//         logger.log("Time: " + now);
//         lastTime = now;
//      }
      //return now - startTime < 3000; // run for 3 seconds
      return false;
   }
}

