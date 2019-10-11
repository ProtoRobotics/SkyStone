package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Mast
{
    private OpMode teleOpClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    public Mast(OpMode teleOpClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.teleOpClass = teleOpClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        init();
    }

    public void init()
    {

    }


    public void doLoop()
    {
        if (Math.abs(gamepad2.left_stick_y)>=.2)
        {
            double speed = gamepad2.left_stick_y * .5;
            this.moveSpeed(speed);
        }
    }

    public void moveSpeed(double speed)
    {
        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastVertical.setPower(speed);

    }

    public void moveCounts(int counts, double speed)
    {
        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mastVertical.setTargetPosition(counts);
        robot.mastVertical.setPower(speed);
    }

    public void rotateCounts(int counts, double speed)
    {
        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastRotator.setPower(speed);
    }
    //Move counts
    //Rotate (degrees) positive right, negative left
}
