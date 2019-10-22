package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop 2019-2020")
public class Teleop2019 extends OpMode
{
    Base base;
    Collector collector;
    Mast mast;
    //Arm

    HardwareMecanum robot;

    @Override
    public void init()
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);

        base = new Base(this, robot, gamepad1, gamepad2);
        collector = new Collector(this, robot, gamepad1, gamepad2);
        mast = new Mast(this, robot, gamepad1, gamepad2);
    }

    @Override
    public void loop()
    {
        try
        {
            base.doLoop();
            collector.doLoop();
            mast.doLoop();
        }
        catch (NullPointerException e)
        {
            //If missing hardware is not connected, ignore it.
        }
    }
}
