package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Disabled
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
        telemetry.addData("Gripper Left AutonomousPosition: ", robot.leftGripper.getPosition());
        telemetry.addData("Gripper Right AutonomousPosition: ", robot.rightGripper.getPosition());
        telemetry.addData("Gripper Rotator Posititon: ", robot.gripperRotator.getPosition());

        telemetry.addData("Left flapper: ", robot.leftFlapper.getPosition());
        telemetry.addData("Right flapper: ", robot.rightFlapper.getPosition());
        telemetry.update();

        double changeInPosition = .3/270.0;

        if (gamepad1.a)
        {
            robot.leftGripper.setPosition(robot.leftGripper.getPosition() + changeInPosition);
        }
        if (gamepad1.b)
        {
            robot.leftGripper.setPosition(robot.leftGripper.getPosition() - changeInPosition);
        }

        if (gamepad1.x)
        {
            robot.rightGripper.setPosition(robot.rightGripper.getPosition() + changeInPosition);
        }
        if (gamepad1.y)
        {
            robot.rightGripper.setPosition(robot.rightGripper.getPosition() - changeInPosition);
        }

        if (gamepad1.left_bumper)
        {
            robot.leftFlapper.setPosition(robot.leftFlapper.getPosition() - changeInPosition);
        }
        if (gamepad1.right_bumper)
        {
            robot.leftFlapper.setPosition(robot.leftFlapper.getPosition() + changeInPosition);
        }

        if (gamepad1.dpad_down)
        {
            robot.rightFlapper.setPosition(robot.rightFlapper.getPosition() - changeInPosition);
        }
        if (gamepad1.dpad_up)
        {
            robot.rightFlapper.setPosition(robot.rightFlapper.getPosition() + changeInPosition);
        }

        if (gamepad1.dpad_left)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() - changeInPosition);
        }
        if (gamepad1.dpad_right)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() + changeInPosition);
        }
    }
}
