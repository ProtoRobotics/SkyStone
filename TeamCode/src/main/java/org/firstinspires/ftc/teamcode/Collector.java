package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Collector
{
    OpMode opModeClass; //Used for telemetry.
    HardwareMecanum robot; //Used to move robot parts.

    Gamepad gamepad1; //Driver
    Gamepad gamepad2; //Gunner

    final double COLLECTOR_SPEED = 1;
    final double COLLECTOR_OFF_SPEED = 0;

    final double LEFT_FLAPPER_OPEN = 0.5;
    final double RIGHT_FLAPPER_OPEN = 0.5;
    final double LEFT_FLEPPER_CLOSED = 0.5;
    final double RIGHT_FLAPPER_CLOSED = 0.5;

    public Collector(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    //
    public void init() {

    }

    public void doLoop()
    {
        if (gamepad1.right_trigger > 0) //intake
        {
            opModeClass.telemetry.addLine("Hello, you are pressing the right trigger button.");
            robot.leftCollector.setPower(-COLLECTOR_SPEED);
            robot.rightCollector.setPower(COLLECTOR_SPEED);
        }
        else if (gamepad1.left_trigger > 0) //outtake
        {
            robot.leftCollector.setPower(COLLECTOR_SPEED);
            robot.rightCollector.setPower(-COLLECTOR_SPEED);
        }
        else if (gamepad1.right_trigger == 0)
        {
            robot.leftCollector.setPower(COLLECTOR_OFF_SPEED);
            robot.rightCollector.setPower(COLLECTOR_OFF_SPEED);
        }
        else if (gamepad1.left_trigger == 0)
        {
            robot.leftCollector.setPower(COLLECTOR_OFF_SPEED);
            robot.rightCollector.setPower(COLLECTOR_OFF_SPEED);
        }
        if (gamepad1.left_bumper)
        {
            robot.leftFlapper.setPosition(LEFT_FLAPPER_OPEN);
            robot.leftFlapper.setPosition(RIGHT_FLAPPER_OPEN);
        }
        if (gamepad1.right_bumper)
        {
            robot.rightFlapper.setPosition(LEFT_FLEPPER_CLOSED);
            robot.rightFlapper.setPosition(RIGHT_FLAPPER_CLOSED);
        }

    }
}
