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


    public Collector(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        init();
    }

    //
    public void init() {

    }

    public void doLoop() {
        if (gamepad1.right_trigger > 0) //intake
        {
            opModeClass.telemetry.addLine("Hello, you are pressing the right trigger button.");
            opModeClass.telemetry.update();
            robot.leftCollector.setPower(COLLECTOR_SPEED);
            robot.rightCollector.setPower(-COLLECTOR_SPEED);
        }
        else if (gamepad1.left_trigger > 0) //outtake
        {
            robot.leftCollector.setPower(-COLLECTOR_SPEED);
            robot.rightCollector.setPower(COLLECTOR_SPEED);
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

    }
}
