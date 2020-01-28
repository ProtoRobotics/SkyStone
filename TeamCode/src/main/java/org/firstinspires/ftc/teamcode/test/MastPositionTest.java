package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMecanum;

import static java.lang.Math.abs;
import static java.lang.Math.subtractExact;

@TeleOp(name="MastPositionTest")
public class MastPositionTest extends OpMode
{
    HardwareMecanum robot;

    @Override
    public void init()
    {
        this.robot = new HardwareMecanum();
        robot.init(hardwareMap);

        robot.mastVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.mastRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop()
    {
        if (abs(gamepad1.left_stick_y) >= .05)
        {
            double speed = -gamepad1.left_stick_y;
            robot.mastVertical.setPower(speed);
        }
        else
        {
            robot.mastVertical.setPower(0);
        }

        if (abs(gamepad1.right_stick_y) >= .05)
        {
            robot.armExtender.setPower(gamepad1.right_stick_y * .5);
        }
        else
        {
            robot.armExtender.setPower(0);
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0)
        {
            robot.leftCollector.setPower(1);
            robot.rightCollector.setPower(1);
        }

        telemetry.addData("Left Collector", robot.leftCollector.getCurrentPosition());
        telemetry.addData("Right Collector", robot.rightCollector.getCurrentPosition());
        telemetry.addData("MastRotator", robot.mastRotator.getCurrentPosition());
        telemetry.addData("MastVertical", robot.mastVertical.getCurrentPosition());
        telemetry.update();
    }
}
