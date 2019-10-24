package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import static java.lang.Thread.sleep;

public class Base
{
    private OpMode teleOpClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    public static final int COUNTS_PER_INCH = 10; //TODO

    public Base(OpMode teleOpClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
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
        double lFrontDrive = 0;
        double rFrontDrive = 0;
        double lRearDrive = 0;
        double rRearDrive = 0;

        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_y) > 0) // forward/reverse
        {
            lFrontDrive = gamepad1.left_stick_y;
            lRearDrive = gamepad1.left_stick_y;
            rFrontDrive = -gamepad1.right_stick_y;
            rRearDrive = -gamepad1.right_stick_y;
        }
        else if (Math.abs(gamepad1.left_stick_x) > 0) // left/right shift
        {
            lFrontDrive = gamepad1.left_stick_x;
            lRearDrive = -gamepad1.left_stick_x;
            rFrontDrive = gamepad1.left_stick_x;
            rRearDrive = -gamepad1.left_stick_x;
        }

        robot.leftFront.setPower(lFrontDrive);
        robot.rightFront.setPower(rFrontDrive);
        robot.leftBack.setPower(lRearDrive);
        robot.rightBack.setPower(rRearDrive);
    }

    /*
        Drive the robot using motor counts.

        The sequential argument determines whether or not to use the Thread.sleep() method.
        This allows us to wait to perform other actions until the move is complete, but also
        gives us the option to disable this feature.
     */
    public void encoderDriveCounts(int leftCounts, int rightCounts, double power, boolean sequential) throws InterruptedException
    {
        int leftBackTarget = robot.leftBack.getCurrentPosition() + leftCounts;
        int leftFrontTarget = robot.leftFront.getCurrentPosition() + leftCounts;
        int rightBackTarget = robot.rightBack.getCurrentPosition() + leftCounts;
        int rightFrontTarget = robot.rightFront.getCurrentPosition() + leftCounts;

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftBack.setTargetPosition(leftBackTarget);
        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.rightBack.setTargetPosition(rightBackTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);

        robot.leftBack.setPower(power);
        robot.leftFront.setPower(power);
        robot.rightBack.setPower(power);
        robot.rightFront.setPower(power);

        //While the motors are still running.
        while (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy())
        {
            //Wait .1 seconds before executing anything else.
            sleep(100);
        }

        //Stop robot.
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
    }

    public void encoderDriveInches(double leftInches, double rightInches, double power, boolean sequential) throws InterruptedException
    {
        encoderDriveCounts((int) leftInches * COUNTS_PER_INCH, (int) rightInches * COUNTS_PER_INCH, power, sequential);
    }

    public void rotateIMU(double degrees) //Positive = clockwise, negative = counter-clockwise
    {

    }

    public void rotateDegreesEncoder(double degrees)
    {

    }
}
