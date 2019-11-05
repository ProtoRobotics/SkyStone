package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Mast
{
    private OpMode opModeClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    public Mast(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
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
        /*
        if (Math.abs(gamepad2.left_stick_y) >= .15)
        {
            double speed = gamepad2.left_stick_y * .5;
            this.moveSpeed(speed);
        }
        else
        {
            this.moveSpeed(0);
        }
         */
        if (Math.abs(gamepad2.right_stick_x) >= .15)
        {
            opModeClass.telemetry.update();
            double speed = gamepad2.right_stick_x * .5;
            rotateSpeed(speed);
        }
        else
        {
            opModeClass.telemetry.update();
            rotateSpeed(0);
        }
    }

    public void moveSpeed(double speed)
    {
        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastVertical.setPower(speed);
    }
    //Move counts
    public void moveCounts(int counts, double speed)
    {
        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mastVertical.setTargetPosition(counts);
        robot.mastVertical.setPower(speed);
    }
    //Rotate (degrees) positive right, negative left
    public void rotateCounts(int counts, double speed)
    {
        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastRotator.setTargetPosition(counts);
        robot.mastRotator.setPower(speed);
    }

    public void rotateSpeed(double speed)
    {
        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastRotator.setPower(speed);
    }
}
