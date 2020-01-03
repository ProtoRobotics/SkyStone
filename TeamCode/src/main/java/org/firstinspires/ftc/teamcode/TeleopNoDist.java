package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop No Dist")
public class TeleopNoDist extends OpMode
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
        mast = new Mast(this, robot, gamepad1, gamepad2, false);
        arm = new Arm(this, robot, gamepad1, gamepad2, false);
    }


    boolean started = false;

    @Override
    public void loop()
    {
        if (!started) //This code runs once after the start button is pressed.
        {
            arm.init();
            mast.init();
            started = true;
        }

        base.doLoop();
        collector.doLoop();
        mast.doLoop();
        arm.doLoop();
        telemetry.update();
    }
}
