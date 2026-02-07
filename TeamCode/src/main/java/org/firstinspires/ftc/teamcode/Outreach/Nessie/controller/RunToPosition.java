package org.firstinspires.ftc.teamcode.Outreach.Nessie.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Outreach.Nessie.controller.DriveController;

public class RunToPosition extends DriveController
{

    public final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    Telemetry telemetry;

    public RunToPosition(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super(hardwareMap);
        this.telemetry = telemetry;
    }


    /**
     * Drives the bot forward or backward in a straight line.
     * @param target distance in inches to travel.
     * @param speed double value indicating the speed from 0 to 1.
     */
    public void forward(int target, double speed)
    {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setTargetPosition(-target);
        rf.setTargetPosition(-target);
        lb.setTargetPosition(-target);
        rb.setTargetPosition(-target);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        while (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())
        {
            telemetry.addData("pos", lf.getCurrentPosition());
            telemetry.addData("pos", rf.getCurrentPosition());
            telemetry.addData("pos", lb.getCurrentPosition());
            telemetry.addData("pos", rb.getCurrentPosition());
            telemetry.update();
        }

        this.driveMotors(0, 0, 0, 0);

    }

    /**
     * Drives the bot forward or backward in a straight line.
     * @param target distance in inches to travel.
     * @param speed double value indicating the speed from 0 to 1.
     */
    public void turn(int target, double speed)
    {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setTargetPosition(-target);
        rf.setTargetPosition(target);
        lb.setTargetPosition(-target);
        rb.setTargetPosition(target);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        while (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())
        {
            telemetry.addData("pos", lf.getCurrentPosition());
            telemetry.addData("pos", rf.getCurrentPosition());
            telemetry.addData("pos", lb.getCurrentPosition());
            telemetry.addData("pos", rb.getCurrentPosition());
            telemetry.update();
        }

        this.driveMotors(0, 0, 0, 0);

    }

}
