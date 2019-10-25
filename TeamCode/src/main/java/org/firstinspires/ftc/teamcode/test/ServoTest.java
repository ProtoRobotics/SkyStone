package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMecanum;

@TeleOp(name="ServoTest")
public class ServoTest extends OpMode
{
    HardwareMecanum robot;

    @Override
    public void init()
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        telemetry.addData("Servo Position:",robot.hook.getPosition());
        telemetry.update();

        double one_degree = 1.0/270.0;

        if (gamepad1.dpad_up)
        {
            robot.hook.setPosition(robot.hook.getPosition() + one_degree);
        }

        if (gamepad1.dpad_down)
        {
            robot.hook.setPosition(robot.hook.getPosition() - one_degree);
        }
    }
}
