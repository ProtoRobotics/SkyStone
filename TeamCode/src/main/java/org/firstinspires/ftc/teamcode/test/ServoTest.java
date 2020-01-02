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
        telemetry.addData("Gripper Position: ", robot.gripperLeft.getPosition());
        telemetry.addData("Gripper Rotator Position: ", robot.gripperRotator.getPosition());

        telemetry.addData("Gripper Position: ", robot.leftFlapper.getPosition());
        telemetry.addData("Gripper Rotator Position: ", robot.rightFlapper.getPosition());
        telemetry.update();

        double changeInPosition = .3/270.0;

        if (gamepad1.a)
        {
            robot.gripperLeft.setPosition(robot.gripperLeft.getPosition() + changeInPosition);
        }

        if (gamepad1.b)
        {
            robot.gripperLeft.setPosition(robot.gripperLeft.getPosition() - changeInPosition);
        }

        if (gamepad1.x)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() + changeInPosition);
        }

        if (gamepad1.y)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() - changeInPosition);
        }
        if (gamepad1.left_bumper)
        {
            robot.leftFlapper.setPosition(robot.leftFlapper.getPosition() - changeInPosition);
        }
        if (gamepad1.right_bumper)
        {
            robot.leftFlapper.setPosition(robot.leftFlapper.getPosition() - changeInPosition);
        }
        if (gamepad1.a)
        {
            robot.rightFlapper.setPosition(robot.rightFlapper.getPosition() - changeInPosition);
        }
        if (gamepad1.b)
        {
            robot.rightFlapper.setPosition(robot.rightFlapper.getPosition() - changeInPosition);
        }
    }
}
