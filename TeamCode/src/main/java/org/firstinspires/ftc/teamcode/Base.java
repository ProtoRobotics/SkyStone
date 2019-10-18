package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_y) > 0)
        { // forward/reverse
            lFrontDrive = -gamepad1.left_stick_y;
            lRearDrive = -gamepad1.left_stick_y;
            rFrontDrive = gamepad1.right_stick_y;
            rRearDrive = gamepad1.right_stick_y;
        }
        else if (Math.abs(gamepad1.left_stick_x) > 0)
        { // left/right shift
            lFrontDrive = -gamepad1.left_stick_y;
            lRearDrive = gamepad1.left_stick_y;
            rFrontDrive = -gamepad1.left_stick_y;
            rRearDrive = gamepad1.left_stick_y;
        }

        robot.leftBack.setPower(lFrontDrive);
        robot.rightFront.setPower(rFrontDrive);
        robot.leftBack.setPower(lRearDrive);
        robot.rightBack.setPower(rRearDrive);
    }

    public void encoderDriveInches(double leftInches, double rightInches, double power)
    {
        encoderDriveCounts((int) leftInches * COUNTS_PER_INCH, (int) rightInches * COUNTS_PER_INCH, power);
    }

    public void encoderDriveCounts(int leftCounts, int rightCounts, double power)
    {
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftBack.setTargetPosition(leftCounts);
        robot.leftFront.setTargetPosition(leftCounts);
        robot.rightBack.setTargetPosition(rightCounts);
        robot.rightFront.setTargetPosition(rightCounts);

        robot.leftBack.setPower(power);
        robot.leftFront.setPower(power);
        robot.rightBack.setPower(power);
        robot.rightFront.setPower(power);
    }

    public void rotateIMU(double degrees) //Positive = clockwise, negative = counter-clockwise
    {

    }

    public void rotateDegreesEncoder(double degrees)
    {

    }
}
