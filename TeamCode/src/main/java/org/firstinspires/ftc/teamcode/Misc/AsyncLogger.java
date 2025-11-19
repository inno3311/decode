package org.firstinspires.ftc.teamcode.Misc;


import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.*;
import java.util.concurrent.ConcurrentLinkedQueue;

public class AsyncLogger {
    private static AsyncLogger instance;
    private final ConcurrentLinkedQueue<String> logQueue = new ConcurrentLinkedQueue<>();
    private FileWriter writer;
    private Thread worker;
    private volatile boolean running = false;

    private AsyncLogger() {}

    public static AsyncLogger getInstance() {
        if (instance == null) {
            instance = new AsyncLogger();
        }
        return instance;
    }

    /** Initialize log file and start background writer thread */
    public void start(String filename) {
        if (running) return;

        try {
            File file = new File(AppUtil.FIRST_FOLDER, filename);
            writer = new FileWriter(file, true); // append mode
            running = true;

            worker = new Thread(() -> {
                while (running) {
                    try {
                        while (!logQueue.isEmpty()) {
                            String msg = logQueue.poll();
                            if (msg != null) {
                                writer.write(msg + "\n");
                            }
                        }
                        writer.flush();
                        Thread.sleep(150); // write every ~0.15s
                    } catch (Exception e) {
                        RobotLog.ee("AsyncLogger", e, "Writer loop error");
                    }
                }

                // final flush when stopped
                try {
                    writer.flush();
                    writer.close();
                } catch (IOException e) {
                    RobotLog.ee("AsyncLogger", e, "Failed to close writer");
                }
            });
            worker.start();

            RobotLog.d("AsyncLogger started: " + file.getAbsolutePath());

        } catch (IOException e) {
            RobotLog.ee("AsyncLogger", e, "Failed to start logger");
        }
    }

    /** Queue a log message (non-blocking) */
    public void log(String msg) {
        if (running) {
            logQueue.add(System.currentTimeMillis() + ": " + msg);
        }
    }

    /** Stop the background thread and close file */
    public void stop() {
        running = false;
        try {
            if (worker != null) {
                worker.join(500); // wait up to 0.5s for shutdown
            }
        } catch (InterruptedException ignored) {}
    }
}

