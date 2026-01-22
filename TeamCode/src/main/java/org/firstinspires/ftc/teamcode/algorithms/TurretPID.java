package org.firstinspires.ftc.teamcode.algorithms;

public class TurretPID
{
    public double kP, kI, kD;
    private double integral = 0;
    private double lastError = 0;

    public TurretPID(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current)
    {
        double error = target - current;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void reset()
    {
        integral = 0;
        lastError = 0;
    }

    public void setPID(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

}
