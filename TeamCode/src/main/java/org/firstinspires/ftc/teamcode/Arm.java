package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.concurrent.TimeUnit;

public class Arm
{
    private OpMode opModeClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    public static final double GRIPPER_ROTATOR_VERTICAL = .76; //Also the gripper rotator initialization point
    public static final double GRIPPER_ROTATOR_HORIZONTAL = .36;
    public static final double GRIPPER_ROTATOR_MAST_LEFT = .26; //Grip rotator pos used in autonomous for left skystone.
    public static final double GRIPPER_ROTATOR_MAST_RIGHT = .51; //Grip rotator pos used in autonomous for right skystone.
    final double GRIPPER_ROTATOR_SPEED = 1.5 / 280;

    public final double GRIPPER_LEFT_OPEN = .49;
    public final double GRIPPER_RIGHT_OPEN = .53;
    public final double GRIPPER_LEFT_CLOSED = .69;
    public final double GRIPPER_RIGHT_CLOSED = .35;
    public final double GRIPPER_LEFT_CAPSTONE = .28;
    public final double GRIPPER_RIGHT_CAPSTONE = .74;

    public Arm(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initTeleop()
    {
        //robot.armDistanceSensor.initialize();
        robot.gripperRotator.setPosition(robot.gripperRotator.getPosition()); //TODO Does this line actually work?
    }

    public void doLoop()
    {
        opModeClass.telemetry.addData("right stick y", gamepad2.right_stick_y);

        if (gamepad2.right_stick_y > .1) //arm out
        {
            robot.armExtender.setPower(getAdjustedSpeed(1.0 * gamepad2.right_stick_y));
        }
        else if (gamepad2.right_stick_y < .1) //arm in
        {
            robot.armExtender.setPower(getAdjustedSpeed(1.0 * gamepad2.right_stick_y));
        }
        else
        {
            robot.armExtender.setPower(0);
        }
        opModeClass.telemetry.addData("Servo position", robot.gripperRotator.getPosition());

        if (gamepad2.left_bumper)
        {
            robot.leftGripper.setPosition(GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(GRIPPER_RIGHT_OPEN);
        }
        else if (gamepad2.right_bumper)
        {
            robot.leftGripper.setPosition(GRIPPER_LEFT_CLOSED);
            robot.rightGripper.setPosition(GRIPPER_RIGHT_CLOSED);
        }

        if (gamepad2.dpad_up)
        {
            robot.gripperRotator.setPosition(GRIPPER_ROTATOR_VERTICAL);
        }
        else if(gamepad2.dpad_down)
        {
            robot.gripperRotator.setPosition(GRIPPER_ROTATOR_HORIZONTAL);
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
            robot.leftGripper.setPosition(GRIPPER_LEFT_CAPSTONE);
            robot.rightGripper.setPosition(GRIPPER_RIGHT_CAPSTONE);
        }

        //opModeClass.telemetry.addData("Arm Distance = ", robot.armDistanceSensor.getDistance(DistanceUnit.CM));
        opModeClass.telemetry.addData("Gripper AutonomousPosition = ", robot.gripperRotator.getPosition());
    }

    //This method will return an adjusted vertical speed based on how far away the arm is from the mast.
    public double getAdjustedSpeed(double speed)
    {
        final int MIN_STOP_COUNTS = 0;
        final int MIN_THROTTLE_COUNTS = 8000;
        final int MAX_THROTTLE_COUNTS = 48000;
        final int MAX_STOP_COUNTS = 56000;

        boolean goingOut = true;
        if (speed > 0)
        {
            goingOut = false;
        }

        int currentCount = robot.rightCollector.getCurrentPosition();

        //We have to check which direction we are going so that we can reverse course after throttling the mast.
        if (currentCount < MIN_STOP_COUNTS && !goingOut)
            return 0; //Stop mast if it is
        if (currentCount < MIN_THROTTLE_COUNTS && !goingOut)
            return (speed / 2.0);
        if (currentCount > MAX_STOP_COUNTS && goingOut)
            return 0;
        if (currentCount > MAX_THROTTLE_COUNTS && goingOut)
            return (speed / 2.0);

        return speed;
    }

    public void moveToPosition(final int targetPosition, double speed, boolean sequential) throws InterruptedException
    {
        final int ERROR_THRESHOLD = 1000;

        final boolean extendingOutwards = (targetPosition > robot.rightCollector.getCurrentPosition()); //is arm moving out or in
        speed = extendingOutwards ? (-Math.abs(speed)) : (Math.abs(speed)); //If going out, speed = negative, retracting = positive speed.
        robot.armExtender.setPower(speed);

        if (sequential) //Run while loop in main thread
        {
            boolean reached = false;
            while (!reached)
            {
                reached = extendingOutwards ? (robot.rightCollector.getCurrentPosition() > targetPosition) : (robot.rightCollector.getCurrentPosition() < targetPosition);
                Thread.sleep(100);
            }
            robot.armExtender.setPower(0);
        }
        else //Run while loop in separate thread to allow other processes to begin.
        {
            new Thread(new Runnable() {

                @Override
                public void run()
                {
                    boolean reached = false;
                    while (!reached)
                    {
                        try
                        {
                            reached = extendingOutwards ? (robot.rightCollector.getCurrentPosition() > targetPosition) : (robot.rightCollector.getCurrentPosition() < targetPosition);
                            Thread.sleep(100);
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                        }
                    }
                    robot.armExtender.setPower(0);
                }
            }).start();
        }
    }

    @Deprecated
    public void moveSeconds(double seconds, double speed)
    {
        robot.armExtender.setPower(speed);
        Runnable armTask = new Runnable()
        {
            public void run()
            {
                robot.armExtender.setPower(0);
                opModeClass.telemetry.addData("Arm position ", robot.rightCollector.getCurrentPosition());
                opModeClass.telemetry.update();
            }
        };

        robot.scheduler.schedule(armTask, (long) (seconds * 1000), TimeUnit.MILLISECONDS);
    }
}
