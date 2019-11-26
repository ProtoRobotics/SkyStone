package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ImuRotator
{
    OpMode opModeClass;
    HardwareMecanum robot;

    Orientation angles;

    int HEADING_THRESHOLD = 1;
    double ERROR_THRESHOLD = 40;
    double MIN_SPEED = 0.1;

    public ImuRotator(OpMode opMode, HardwareMecanum robot)
    {
        this.opModeClass = opMode;
        this.robot = robot;
    }

    public ImuRotator(OpMode opMode, HardwareMecanum robot, boolean initializeIMU)
    {
        this.opModeClass = opMode;
        this.robot = robot;
        if (initializeIMU)
        {
            initializeIMU();
        }
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

    public void rotateIMU(double speed, double angle)
    {
        angles = robot.imu.getAngularOrientation();
        double targetAngle = angles.firstAngle - angle;

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This will continuously execute onHeading until the robot is within a 1 degree threshold of the target.
        while (!onHeading(speed, targetAngle))
        {
            opModeClass.telemetry.update();
        }
    }

    public void rotateToAngle(double speed, double angle)
    {
        angles = robot.imu.getAngularOrientation();
        double targetAngle = -angle;

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This will continuously execute onHeading until the robot is within a 1 degree threshold of the target.
        while (!onHeading(speed, targetAngle))
        {
            opModeClass.telemetry.update();
        }
    }

    public boolean onHeading(double speed, double angle)
    {
        double error;
        boolean onTarget = false;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            speed = 0;
            onTarget = true;
        }
        else
        {
            if (error < 0)
                speed *= -1;

            if (Math.abs(error) <= ERROR_THRESHOLD * 0.25)
                speed = (speed > 0) ? MIN_SPEED : -(MIN_SPEED);
            else if (Math.abs(error) <= ERROR_THRESHOLD * 0.50)
                speed = (speed > 0) ? MIN_SPEED + 0.05 : -(MIN_SPEED + 0.05);
            else if (Math.abs(error) <= ERROR_THRESHOLD * 0.75)
                speed = (speed > 0) ? MIN_SPEED + 0.1 : -(MIN_SPEED + 0.1);
            else if (Math.abs(error) <= ERROR_THRESHOLD)
                speed = (speed > 0) ? MIN_SPEED + 0.15 : -(MIN_SPEED + 0.15);
        }

        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);

        opModeClass.telemetry.addData("speed", speed);
        opModeClass.telemetry.addData("onTarget", onTarget);
        return onTarget;
    }

    public double getError(double targetAngle)
    {
        double robotError;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;

        opModeClass.telemetry.addData("targetAngle", targetAngle);
        opModeClass.telemetry.addData("angles.firstAngle", angles.firstAngle);
        opModeClass.telemetry.addData("Error", robotError);
        opModeClass.telemetry.addData("Error Coterminal", "" + getCoterminalAngle(robotError));
        return getCoterminalAngle(robotError);
    }


    //Returns the shortest angle coterminal to the input. If 240 is input as the angle value, returns -120.
    public double getCoterminalAngle(double angle)
    {
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
