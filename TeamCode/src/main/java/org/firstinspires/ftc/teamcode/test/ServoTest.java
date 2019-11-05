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
        telemetry.addData("Gripper Position: ", robot.gripper.getPosition());
        telemetry.addData("Gripper Rotator Position: ", robot.gripperRotator.getPosition());
        telemetry.update();

        double changeInPosition = .3/270.0;

        if (gamepad1.a)
        {
            robot.gripper.setPosition(robot.gripper.getPosition() + changeInPosition);
        }

        if (gamepad1.b)
        {
            robot.gripper.setPosition(robot.gripper.getPosition() - changeInPosition);
        }

        if (gamepad1.x)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() + changeInPosition);
        }

        if (gamepad1.y)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() - changeInPosition);
        }
    }
}
