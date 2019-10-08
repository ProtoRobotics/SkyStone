package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
        base.doLoop();
        collector.doLoop();
        mast.doLoop();
    }
}
