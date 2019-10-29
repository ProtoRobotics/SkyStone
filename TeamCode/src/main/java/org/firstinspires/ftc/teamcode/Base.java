package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import static java.lang.Thread.sleep;

public class Base
{
    private OpMode opModeClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    public static final double WHEEL_DIAMETER = 100.0/25.4; //Wheels are 100mm, converted to inches is ~4.
    public static final int COUNTS_PER_INCH = (int) (784.0 / WHEEL_DIAMETER);

    public Base(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
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

    public void crabsteer(double inches, double power)  throws InterruptedException//TODO add power
    {
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double powerFinal = Math.abs(power);

        double countsToMove = inches * COUNTS_PER_INCH;

        //if (direction == 0) {
            robot.leftFront.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() + countsToMove));
            robot.leftBack.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() - countsToMove));
            robot.rightFront.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() + countsToMove));
            robot.rightBack.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() - countsToMove));

            robot.leftFront.setPower(powerFinal);
            robot.leftBack.setPower(-powerFinal);
            robot.rightFront.setPower(powerFinal);
            robot.rightBack.setPower(-powerFinal);
        /*
        }
        else if (direction == 1) {
            robot.leftFront.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() - countsToMove));
            robot.leftBack.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() + countsToMove));
            robot.rightFront.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() - countsToMove));
            robot.rightBack.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() + countsToMove));

            robot.leftFront.setPower(-powerFinal);
            robot.leftBack.setPower(powerFinal);
            robot.rightFront.setPower(-powerFinal);
            robot.rightBack.setPower(powerFinal);
        }
        */
        //autonomousClass.telemetry.addData("is it running?", "YEAH!");

        //autonomousClass.telemetry.addData("EXITING", "YEAH!");
        Thread.sleep(1000);
    }
}
