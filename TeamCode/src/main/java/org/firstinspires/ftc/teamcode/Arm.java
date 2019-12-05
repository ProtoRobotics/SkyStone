package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm
{
    private OpMode opModeClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    // final double EXTENDER_MAX = .78;
    //final double EXTENDER_MIN = .21; //.21 is furthest physically possible
    final double EXTENSION_MAX = 3;  //MAX DISTANCE ARM (INCHES) CAN BE EXTENDED MEASURED BY ARM DISTANCE SENSOR
    final double EXTENSION_MIN = 12;  //MIN DISTANCE ARM (INCHES) CAN BE EXTENDED MEASURED BY ARM DISTANCE SENSOR
    final double EXTENDER_RATE_OF_CHANGE = 1.5 / 280;

    final double GRIPPER_ROTATOR_POS_1 = .12; //Also the gripper rotator initialization point
    final double GRIPPER_ROTATOR_POS_2 = .51;
    final double GRIPPER_ROTATOR_SPEED = 1.5 / 280;

    final double GRIPPER_OPEN = .29;
    final double GRIPPER_CLOSE = .55;
    final double GRIPPER_INITALIZATION_POINT = .332; //Where we want the gripper to be on init

    public Arm(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init()
    {
        //robot.gripper.setPosition(GRIPPER_INITALIZATION_POINT);
        //robot.gripperRotator.setPosition(GRIPPER_ROTATOR_POS_1);
        //robot.armExtender.setPosition(EXTENDER_MIN);
        //robot.armExtender.setPosition(robot.armExtender.getPosition());

        robot.armDistanceSensor.initialize();
        robot.gripperRotator.setPosition(robot.gripperRotator.getPosition());
        robot.gripper.setPosition(robot.gripper.getPosition());
    }

    public void doLoop()
    {
        if (gamepad2.x)
        {
            robot.armExtender.setPower(.3);
        }
        else if (gamepad2.b)
        {
            robot.armExtender.setPower(-.3);
        }
        else
        {
            robot.armExtender.setPower(0);
        }

        opModeClass.telemetry.addData("Servo position", robot.gripperRotator.getPosition());
        opModeClass.telemetry.update();

        if (gamepad2.left_bumper)
        {
            robot.gripper.setPosition(GRIPPER_CLOSE);
        }
        else if (gamepad2.right_bumper)
        {
            robot.gripper.setPosition(GRIPPER_OPEN);
        }

        if (gamepad2.dpad_down)
        {
            robot.gripperRotator.setPosition(GRIPPER_ROTATOR_POS_1);
        }
        else if(gamepad2.dpad_up)
        {
            robot.gripperRotator.setPosition(GRIPPER_ROTATOR_POS_2);
        }

        if (gamepad2.dpad_left)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() + GRIPPER_ROTATOR_SPEED);
        }
        else if (gamepad2.dpad_right)
        {
            robot.gripperRotator.setPosition(robot.gripperRotator.getPosition() - GRIPPER_ROTATOR_SPEED);
        }
        opModeClass.telemetry.addData("Arm Distance = ", robot.armDistanceSensor.getDistance(DistanceUnit.CM));
        opModeClass.telemetry.addData("Gripper Position = ", robot.gripperRotator.getPosition());
        opModeClass.telemetry.update();
    }
}
