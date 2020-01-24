package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop 2019-2020")
public class Teleop2019 extends OpMode
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;

    @Override
    public void init()
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);

        base = new Base(this, robot, gamepad1, gamepad2);
        collector = new Collector(this, robot, gamepad1, gamepad2);
        mast = new Mast(this, robot, gamepad1, gamepad2);
        arm = new Arm(this, robot, gamepad1, gamepad2);
    }

    boolean started = false;

    @Override
    public void loop()
    {
        if (!started) //This code runs once after the start button is pressed.
        {
            arm.initTeleop();
            mast.initTeleop();
            started = true;
        }

        base.doLoop();
        collector.doLoop();
        mast.doLoop();
        arm.doLoop();
        telemetry.update();
    }
}
