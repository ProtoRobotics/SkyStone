package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ImuRotator
{
    HardwareMecanum robot;

    Orientation angles;

    int HEADING_THRESHOLD = 1;
    double ERROR_THRESHOLD = 40;
    double MIN_SPEED = 0.15;

    //TODO Current implementation only supports use of 1 rotation.

    //ImuRotator myImuRotator = new ImuRotator(myRobotVariable);
    public ImuRotator(HardwareMecanum robot)
    {
        this.robot = robot;
    }

    public void rotateIMU(double speed, double angle)
    {
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!onHeading(speed, angle)) {}
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
            else if (Math.abs(error) <= ERROR_THRESHOLD * 0.25)
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

        return onTarget;
    }

    public double getError(double targetAngle)
    {
        double robotError;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;

        return getAbsoluteAngle(robotError);
    }

    public double getAbsoluteAngle(double angle)
    {
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
