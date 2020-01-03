package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

public class Base
{
    private OpMode opModeClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    //public static final double WHEEL_DIAMETER = 100.0/25.4; //Wheels are 100mm, converted to inches is ~4.
    //public static final int COUNTS_PER_INCH = (int) (784.0 / WHEEL_DIAMETER); //784 counts per wheel rotation, divided by diameter yeilds cpi.
    public static final double COUNTS_PER_INCH = 61.38;
    public static final double COUNTS_PER_INCH_CRAB = 1000.0/15; //TODO
    public static final double COUNTS_PER_INCH_DEGREE = 4000;
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_INCH_DEGREE; //TODO


    public static final double HOOK_UP_POSITION = .87;
    public static final double HOOK_DOWN_POSITION = .48;

    public Base(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        initializeIMU();
        initializeEncoders();
    }

    public void initializeIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        robot.imu.initialize(parameters);
    }

    public void initializeEncoders()
    {
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void doLoop()
    {
        double lFrontDrive = 0;
        double rFrontDrive = 0;
        double lRearDrive = 0;
        double rRearDrive = 0;
        double powerReduction = 3.0/4.0;

        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_y) > 0) // forward/reverse
        {
            lFrontDrive = gamepad1.left_stick_y;
            lRearDrive = gamepad1.left_stick_y;
            rFrontDrive = -gamepad1.right_stick_y;
            rRearDrive = -gamepad1.right_stick_y;
        }
        else if (Math.abs(gamepad1.left_stick_x) > 0) // left/right shift
        {
            lFrontDrive = -gamepad1.left_stick_x;
            lRearDrive = gamepad1.left_stick_x;
            rFrontDrive = -gamepad1.left_stick_x;
            rRearDrive = gamepad1.left_stick_x;
        }

        robot.leftFront.setPower(lFrontDrive * powerReduction);
        robot.rightFront.setPower(rFrontDrive * powerReduction);
        robot.leftBack.setPower(lRearDrive * powerReduction);
        robot.rightBack.setPower(rRearDrive * powerReduction);

        if (gamepad1.a)
        {
            hookDown();
        }

        if (gamepad1.b)
        {
            hookUp();
        }

        //opModeClass.telemetry.addData("BASE:  Left Front POS = ", robot.leftFront.getCurrentPosition());
        //opModeClass.telemetry.addData("BASE:  Distance = ", robot.baseDistanceSensor.getDistance(DistanceUnit.CM));
        //Location loc;
        //loc = this.scanStone();
        //opModeClass.telemetry.addData("BASE:  Location = ", loc);
        //opModeClass.telemetry.update();
    }

    /*
        Drive the robot using motor counts.

        The sequential argument determines whether or not to use the Thread.sleep() method.
        This allows us to wait to perform other actions until the move is complete, but also
        gives us the option to disable this feature.
     */
    public void encoderDriveCounts(int leftCounts, int rightCounts, double power, boolean sequential) throws InterruptedException
    {
        int leftBackTarget = robot.leftBack.getCurrentPosition() - leftCounts;
        int leftFrontTarget = robot.leftFront.getCurrentPosition() - leftCounts;
        int rightBackTarget = robot.rightBack.getCurrentPosition() + rightCounts;
        int rightFrontTarget = robot.rightFront.getCurrentPosition() + rightCounts;

        robot.leftBack.setTargetPosition(leftBackTarget);
        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.rightBack.setTargetPosition(rightBackTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);

        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftBack.setPower(power);
        robot.leftFront.setPower(power);
        robot.rightBack.setPower(power);
        robot.rightFront.setPower(power);

        //While the motors are still running.
        if (sequential)
        {
            while (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy())
            {
                //Wait .1 seconds before executing anything else.
                sleep(100);
            }
        }

        //Stop robot.
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
    }

    public void encoderDriveInches(double leftInches, double rightInches, double power, boolean sequential) throws InterruptedException
    {
        encoderDriveCounts((int) (leftInches * COUNTS_PER_INCH), (int) (rightInches * COUNTS_PER_INCH), power, sequential);
    }

    public void rotateDegreesEncoder(double degrees, double power, boolean sequential) throws InterruptedException
    {
        if (degrees > 0) //Clockwise rotation
        {
            encoderDriveCounts((int) (degrees * COUNTS_PER_DEGREE), (int) (-degrees * COUNTS_PER_DEGREE), power, sequential);
        }
        if (degrees < 0) //Counter-clockwise rotation
        {
            encoderDriveCounts((int) (-degrees * COUNTS_PER_DEGREE), (int) (degrees * COUNTS_PER_DEGREE), power, sequential);
        }
    }

    public void encoderCrabsteer(int direction, double inches, double power, boolean sequential) throws InterruptedException //left = 0, right = 1
    {
        //TODO Make signature = double inches, double power, boolean sequential
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double powerFinal = Math.abs(power);

        double countsToMove = inches * COUNTS_PER_INCH_CRAB;

        if (direction == 0)
        {
            robot.leftFront.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() + countsToMove));
            robot.leftBack.setTargetPosition((int) Math.round(robot.leftBack.getCurrentPosition() - countsToMove));
            robot.rightFront.setTargetPosition((int) Math.round(robot.rightFront.getCurrentPosition() + countsToMove));
            robot.rightBack.setTargetPosition((int) Math.round(robot.rightBack.getCurrentPosition() - countsToMove));

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFront.setPower(powerFinal);
            robot.leftBack.setPower(-powerFinal);
            robot.rightFront.setPower(powerFinal);
            robot.rightBack.setPower(-powerFinal);
        }
        else if (direction == 1)
        {
            robot.leftFront.setTargetPosition((int) Math.round(robot.leftFront.getCurrentPosition() - countsToMove));
            robot.leftBack.setTargetPosition((int) Math.round(robot.leftBack.getCurrentPosition() + countsToMove));
            robot.rightFront.setTargetPosition((int) Math.round(robot.rightFront.getCurrentPosition() - countsToMove));
            robot.rightBack.setTargetPosition((int) Math.round(robot.rightBack.getCurrentPosition() + countsToMove));

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFront.setPower(-powerFinal);
            robot.leftBack.setPower(powerFinal);
            robot.rightFront.setPower(-powerFinal);
            robot.rightBack.setPower(powerFinal);
        }

        if (sequential)
        {
            while (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy())
            {
                //Wait .1 seconds before executing anything else.
                sleep(100);
            }
        }
    }

    public void hookUp()
    {

        robot.hook.setPosition(HOOK_UP_POSITION);
    }

    public void hookDown()
    {

        robot.hook.setPosition(HOOK_DOWN_POSITION);
    }

    public Location scanStone()
    {
        // Read the sensor
        NormalizedRGBA colorsRight = robot.rightColorSensor.getNormalizedColors();
        NormalizedRGBA colorsLeft = robot.leftColorSensor.getNormalizedColors();

        // declare and init booleans
        boolean colorSensorRight;
        boolean colorSensorLeft;

        // The base has 2 color sensors - 1 over left wheel and 1 over right wheel (referenced from driver position)
        // Boolean sensor value is set to 'TRUE' if it detects 'BLACK' (i.e. SkyStone)
        // Based on trials, we find that 2 of the RGB values were consistently above 0.002 when viewing the yellow block
        // whereas all values were around 0.002 when viewing BLACK block (i.e.) SkyStone.

        if(colorsRight.red <= 0.003 && colorsRight.green <= 0.003 && colorsRight.blue <= 0.003){
            colorSensorRight = true;
        }
        else
            colorSensorRight = false;

        if(colorsLeft.red <= 0.003 && colorsLeft.green <= 0.003 && colorsLeft.blue <= 0.003){
            colorSensorLeft = true;
        }
        else
            colorSensorLeft = false;

        opModeClass.telemetry.addLine()
                .addData("BASE: Color Sensor LEFT = r", "%.3f", colorsLeft.red)
                .addData("g", "%.3f", colorsLeft.green)
                .addData("b", "%.3f", colorsLeft.blue);
        opModeClass.telemetry.addLine()
                .addData("BASE: Color Sensor RIGHT = r", "%.3f", colorsRight.red)
                .addData("g", "%.3f", colorsRight.green)
                .addData("b", "%.3f", colorsRight.blue);

        // Based on color found by each sensor, this method outputs the location of the Skystone given
        // row of 3 stones.
        //      0 = left stone
        //      1 = middle stone
        //      2 = right stone        // output location of Sky Stone
        if(colorSensorRight == true && colorSensorLeft == false){
            return Location.RIGHT;
        }
        else if(colorSensorLeft == true && colorSensorRight == false){
            return Location.LEFT;
        }
        else {
            return Location.CENTER;
        }

    }

}



