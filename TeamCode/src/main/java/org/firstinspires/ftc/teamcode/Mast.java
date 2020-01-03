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

    private boolean useDistance = false;

    private final double MIN_STOP_DISTANCE = 5.5;
    private final double MIN_THROTTLE_DISTANCE = 16;
    private final double MAX_STOP_DISTANCE = 60;
    private final double MAX_THROTTLE_DISTANCE = 55;

    private final double MAST_ROTATE_SPEED = .2;
    private final double SPEED = 0.5;
    private final double LEFT_ANGLE = -25;
    private final double LEFT_ARM_LENGTH = 12;
    private final double CENTER_ANGLE = 0;
    private final double CENTER_ARM_LENGTH = 6;
    private final double RIGHT_ANGLE = 25;
    private final double RIGHT_ARM_LENGTH = 12;

    public Mast(OpMode opModeClass, HardwareMecanum robot, Gamepad gamepad1, Gamepad gamepad2, boolean useDistance)
    {
        this.opModeClass = opModeClass;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.useDistance = useDistance;
    }

    public void init()
    {
        //robot.mastDistanceSensor.initialize();
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

        if (gamepad2.b) //rotate right
        {
            rotateSpeed(-MAST_ROTATE_SPEED);
        }
        else if (gamepad2.x) //rotate left
        {
            rotateSpeed(MAST_ROTATE_SPEED);
        }
        else
        {
            rotateSpeed(0);
        }
        //opModeClass.telemetry.addData("Mast Distance = ",robot.mastDistanceSensor.getDistance(DistanceUnit.CM));
    }

    public void moveSpeed(double speed)
    {
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.mastVertical.setPower(speed);

        if (useDistance)
        {
            robot.mastVertical.setPower(getAdjustedSpeed(speed));
        }
        else
        {
            robot.mastVertical.setPower(speed);
        }
    }

    //Move counts
    public void moveCounts(int counts, double speed)
    {
        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mastVertical.setTargetPosition(counts);
        robot.mastVertical.setPower(speed);
    }

    //Rotate (degrees) positive right, negative left
    public void rotateCounts(int counts, double speed)
    {
        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastRotator.setTargetPosition(counts);
        robot.mastRotator.setPower(speed);
    }

    public void rotateSpeed(double speed)
    {
        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mastRotator.setPower(speed);
    }

    //This method will return an adjusted vertical speed based on how far away the arm is from the mast.
    public double getAdjustedSpeed(double speed)
    {
        boolean goingUp = true;
        if (speed < 0)
        {
            goingUp = false;
        }

        double distance = robot.mastDistanceSensor.getDistance(DistanceUnit.CM);

        //We have to check which direction we are going so that we can reverse course after throttling the mast.
        if (distance < MIN_STOP_DISTANCE && !goingUp)
            return 0; //Stop mast if it is
        if (distance < MIN_THROTTLE_DISTANCE && !goingUp)
            return (speed / 3.5);
        if (distance > MAX_STOP_DISTANCE && goingUp)
            return 0;
        if (distance > MAX_THROTTLE_DISTANCE && goingUp)
            return (speed / 3.5);

        return speed;
    }

    public void moveToPos(double pos) {

        double error;
        //double speed = SPEED;

        error = pos - robot.mastDistanceSensor.getDistance(DistanceUnit.CM);
        while (abs(error) > 0.5) {
            if (error < 0) {
                this.moveSpeed(SPEED * -1);
            }
            else {
                this.moveSpeed(SPEED);
            }
            error = pos - robot.mastDistanceSensor.getDistance(DistanceUnit.CM);
        }
    }

    public void setMastOnSkystone(Location loc) {
        switch (loc) {
            case LEFT:
                //set mast and arm to the LEFT position

                //call function to turn mast
                this.moveToPos(25);
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
