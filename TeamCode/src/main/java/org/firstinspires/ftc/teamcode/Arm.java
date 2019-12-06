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

    final double GRIPPER_ROTATOR_POS_1 = .12; //Also the gripper rotator initialization point
    final double GRIPPER_ROTATOR_POS_2 = .51;
    final double GRIPPER_ROTATOR_SPEED = 1.5 / 280;

    final double GRIPPER_OPEN = .542;
    final double GRIPPER_CLOSE = 1.0;

    //TODO Find actual positions.
    private final double MIN_STOP_DISTANCE = 26.0;
    private final double MIN_THROTTLE_DISTANCE = 29.0;
    private final double MAX_STOP_DISTANCE = 43.0;
    private final double MAX_THROTTLE_DISTANCE = 42.0;

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
        if (gamepad2.x) //arm out
        {
            robot.armExtender.setPower(getAdjustedSpeed(-.5));
        }
        else if (gamepad2.b) //arm in
        {
            robot.armExtender.setPower(getAdjustedSpeed(.5));
        }
        else
        {
            robot.armExtender.setPower(0);
        }

        opModeClass.telemetry.addData("Servo position", robot.gripperRotator.getPosition());

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
    }

    //This method will return an adjusted vertical speed based on how far away the arm is from the mast.
    public double getAdjustedSpeed(double speed)
    {
        boolean goingOut = true;
        if (speed > 0)
        {
            goingOut = false;
        }

        double distance = robot.armDistanceSensor.getDistance(DistanceUnit.CM);

        //We have to check which direction we are going so that we can reverse course after throttling the mast.
        if (distance < MIN_STOP_DISTANCE && !goingOut)
            return 0; //Stop mast if it is
        if (distance < MIN_THROTTLE_DISTANCE && !goingOut)
            return (speed / 2.0);
        if (distance > MAX_STOP_DISTANCE && goingOut)
            return 0;
        if (distance > MAX_THROTTLE_DISTANCE && goingOut)
            return (speed / 2.0);

        return speed;
    }
}
