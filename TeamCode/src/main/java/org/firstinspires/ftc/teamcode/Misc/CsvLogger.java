package org.firstinspires.ftc.teamcode.Misc;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * CvsLogger — a singleton CSV logger for FTC data.
 *
 * Usage:
 *   CvsLogger logger = CvsLogger.getInstance();
 *   logger.start("test_log");
 *   logger.log("time_sec,motorPower,heading");
 *   logger.log(String.format("%.3f,%.3f,%.3f", t, pwr, heading));
 *   logger.close();
 */
public class CsvLogger
{
   private static CsvLogger instance;
   private BufferedWriter writer;
   private boolean isActive = false;

   private CsvLogger() { } // private constructor

   /** Get the singleton instance */
   public static CsvLogger getInstance() {
      if (instance == null) {
         instance = new CsvLogger();
      }
      return instance;
   }

   /** Start a new log file (creates folder and file) */
   public void start(String filenameBase) {
      try {
         File logDir = new File("/sdcard/FIRST/logs/");
         if (!logDir.exists()) logDir.mkdirs();

         File logFile = new File(logDir, filenameBase + "_" + System.currentTimeMillis() + ".csv");
         writer = new BufferedWriter(new FileWriter(logFile));

         isActive = true;
         log("### Logging started: " + logFile.getName());
      } catch (IOException e) {
         e.printStackTrace();
         isActive = false;
      }
   }

   /** Log a line of text (adds newline automatically) */
   public void log(String line) {
      if (!isActive || writer == null) return;
      try {
         writer.write(line);
         writer.newLine();
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   /** Flush data to disk (optional, not needed every call) */
   public void flush() {
      if (!isActive || writer == null) return;
      try {
         writer.flush();
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   /** Close the log file */
   public void close() {
      if (!isActive || writer == null) return;
      try {
         writer.flush();
         writer.close();
         isActive = false;
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   /** Check if the logger is active */
   public boolean isActive() {
      return isActive;
   }
}

