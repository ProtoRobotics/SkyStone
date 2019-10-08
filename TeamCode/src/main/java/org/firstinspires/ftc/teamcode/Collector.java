package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.Gamepad;

public class Collector
{
    OpMode teleOpClass; //Used for telemetry.
    HardwareMecanum robot; //Used to move robot parts.

    Gamepad gamepad1; //Driver
    Gamepad gamepad2; //Gunner

    public Collector(OpMode teleOpClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.teleOpClass = teleOpClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        init();
    }

    //
    public void init()
    {

    }

    public void doLoop()
    {
        if (gamepad1.a)
        {
            teleOpClass.telemetry.addLine("Hello, you are pressing the A button.");
            teleOpClass.telemetry.update();


        }
    }
}