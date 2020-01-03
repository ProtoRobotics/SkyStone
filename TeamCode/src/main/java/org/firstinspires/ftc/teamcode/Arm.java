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

    private boolean useDistance;

    final double GRIPPER_ROTATOR_POS_1 = .74; //Also the gripper rotator initialization point
    final double GRIPPER_ROTATOR_POS_2 = .36;
    final double GRIPPER_ROTATOR_SPEED = 1.5 / 280;

    final double GRIPPER_LEFT_OPEN = .43;
    final double GRIPPER_RIGHT_OPEN = .56;
    final double GRIPPER__LEFT_CLOSED = .65;
    final double GRIPPER_RIGHT_CLOSED = .35;
    final double GRIPPER_LEFT_CAPSTONE = .28;
    final double GRIPPER_RIGHT_CAPSTONE = .71;

    private final double MIN_STOP_DISTANCE = 25.3;
    private final double MIN_THROTTLE_DISTANCE = 28.0;
    private final double MAX_STOP_DISTANCE = 43.0;
    private final double MAX_THROTTLE_DISTANCE = 42.0;

    public Arm(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2, boolean useDistance)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.useDistance = useDistance;
    }

    public void initTeleop()
    {
        robot.armDistanceSensor.initialize();
        robot.gripperRotator.setPosition(robot.gripperRotator.getPosition());
    }

    public void doLoop()
    {
        opModeClass.telemetry.addData("right stick y", gamepad2.right_stick_y);

        if (useDistance)
        {
            if (gamepad2.right_stick_y > .1) //arm out
            {
                robot.armExtender.setPower(getAdjustedSpeed(.8 * gamepad2.right_stick_y));
            }
            else if (gamepad2.right_stick_y < .1) //arm in
            {
                robot.armExtender.setPower(getAdjustedSpeed(.8 * gamepad2.right_stick_y));
            }
            else
            {
                robot.armExtender.setPower(0);
            }
        }
        else
        {
            if (gamepad2.right_stick_y > .1) //arm out
            {
                robot.armExtender.setPower(gamepad2.right_stick_y);
            }
            else if (gamepad2.right_stick_y < .1) //arm in
            {
                robot.armExtender.setPower(gamepad2.right_stick_y);
            }
            else
            {
                robot.armExtender.setPower(0);
            }
        }

        opModeClass.telemetry.addData("Servo position", robot.gripperRotator.getPosition());

        if (gamepad2.left_bumper)
        {
            robot.gripperLeft.setPosition(GRIPPER_LEFT_OPEN);
            robot.gripperRight.setPosition(GRIPPER_RIGHT_OPEN);
        }
        else if (gamepad2.right_bumper)
        {
            robot.gripperLeft.setPosition(GRIPPER__LEFT_CLOSED);
            robot.gripperRight.setPosition(GRIPPER_RIGHT_CLOSED);
        }

        if (gamepad2.dpad_up)
        {
            robot.gripperRotator.setPosition(GRIPPER_ROTATOR_POS_1);
        }
        else if(gamepad2.dpad_down)
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

        if (gamepad2.a && gamepad2.y)
        {
            robot.gripperLeft.setPosition(GRIPPER_LEFT_CAPSTONE);
            robot.gripperRight.setPosition(GRIPPER_RIGHT_CAPSTONE);
        }

        //opModeClass.telemetry.addData("Arm Distance = ", robot.armDistanceSensor.getDistance(DistanceUnit.CM));
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
