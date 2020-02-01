package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

public class Mast
{
    private OpMode opModeClass; //Used for telemetry.
    private HardwareMecanum robot; //Used to move robot parts.

    private Gamepad gamepad1; //Driver
    private Gamepad gamepad2; //Gunner

    private final int MIN_COUNTS_MAST_VERT = 0;
    private final int MIN_THROTTLE_COUNTS_MAST_VERT = 500;
    private final int MAX_COUNTS_MAST_VERT = 9500;
    private final int MAX_THROTTLE_COUNTS_MAST_VERT = 9000;

    private final double MAST_ROTATE_SPEED = .2;

    public Mast(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initTeleop()
    {
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetMastEncoders()
    {
        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void doLoop()
    {
        if (abs(gamepad2.left_stick_y) >= .15)
        {
            double speed = -gamepad2.left_stick_y;
            this.moveSpeed(speed);
        }
        else
        {
            this.moveSpeed(0);
        }

        if (gamepad2.right_trigger > 0) //rotate right
        {
            rotateSpeed(-MAST_ROTATE_SPEED);
        }
        else if (gamepad2.left_trigger > 0) //rotate left
        {
            rotateSpeed(MAST_ROTATE_SPEED);
        }
        else
        {
            rotateSpeed(0);
        }
        opModeClass.telemetry.addData("Mast vertical counts: ", robot.mastVertical.getCurrentPosition());
        opModeClass.telemetry.addData("Mast rotator counts: ", robot.mastRotator.getCurrentPosition());
    }

    public void moveSpeed(double speed)
    {
        robot.mastVertical.setPower(getAdjustedSpeedEncoder(speed));
    }

    //Move counts
    public void moveCounts(int counts, double speed)
    {
        robot.mastVertical.setTargetPosition(robot.mastVertical.getCurrentPosition() + counts);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mastVertical.setPower(speed);
    }

    //Rotate (degrees) positive right, negative left
    public void rotateCounts(int counts, double speed)
    {
        robot.mastRotator.setTargetPosition(robot.mastRotator.getCurrentPosition() + counts);
        robot.mastRotator.setPower(speed);
    }

    public void rotateSpeed(double speed)
    {
        robot.mastRotator.setPower(speed);
    }

    public double getAdjustedSpeedEncoder(double speed)
    {
        boolean goingUp = true;
        if (speed < 0)
        {
            goingUp = false;
        }

        int counts = robot.mastVertical.getCurrentPosition();

        if (counts < MIN_COUNTS_MAST_VERT && !goingUp)
            return 0; //Stop mast if it is
        if (counts < MIN_THROTTLE_COUNTS_MAST_VERT && !goingUp)
            return (speed / 3.5);
        if (counts > MAX_COUNTS_MAST_VERT && goingUp)
            return 0;
        if (counts > MAX_THROTTLE_COUNTS_MAST_VERT && goingUp)
            return (speed / 3.5);

        return speed;
    }

    public void setMastOnSkystone(Location loc) {
        switch (loc) {
            case LEFT:
                //set mast and arm to the LEFT position

                //call function to turn mast
               //Not Used: this.moveToPos(25);
                break;
            case CENTER:
                //set mast and arm to the CENTER position
                break;
            case RIGHT:
                //set mast and arm to the RIGHT position
                break;
        }

    }
}
