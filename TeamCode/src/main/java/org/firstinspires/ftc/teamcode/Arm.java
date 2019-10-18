package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm
{
    private OpMode teleOpClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    final double EXTENDER_SPEED = 1;
    final double EXTENDER_OFF_SPEED = 0;

    final double GRIPPER_ROTATOR_POS_1 = 0;
    final double GRIPPER_ROTATOR_POS_2 = 90 / 280;
    final double GRIPPER_ROTATOR_SPEED = 1 / 280;

    final double GRIPPER_OPEN = 1;
    final double GRIPPER_CLOSE = 0;

    public Arm(OpMode teleOpClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.teleOpClass = teleOpClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        init();
    }

    public void init()
    {

    }

    public void doLoop()
    {
        if (gamepad2.x)
        {
            robot.armExtender.setPower(EXTENDER_SPEED);
        }
        else if(gamepad2.b)
        {
            robot.armExtender.setPower(-EXTENDER_SPEED);
        }
        else if(!gamepad2.x && !gamepad2.b)
        {
            robot.armExtender.setPower(EXTENDER_OFF_SPEED);
        }

        if (gamepad2.left_bumper)
        {
            robot.gripper.setPosition(GRIPPER_CLOSE);
        }
        else if (gamepad1.right_bumper)
        {
            robot.gripper.setPosition(GRIPPER_OPEN);
        }

        if (gamepad2.dpad_down)
        {
            robot.gripperRotater.setPosition(GRIPPER_ROTATOR_POS_1);
        }
        else if(gamepad2.dpad_up)
        {
            robot.gripperRotater.setPosition(GRIPPER_ROTATOR_POS_2);
        }

        if (gamepad2.dpad_left)
        {
            robot.gripperRotater.setPosition(robot.gripperRotater.getPosition() + GRIPPER_ROTATOR_SPEED);
        }
        else if (gamepad2.dpad_right)
        {
            robot.gripperRotater.setPosition(robot.gripperRotater.getPosition() - GRIPPER_ROTATOR_SPEED);
        }
    }
}
