package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMecanum;

import static java.lang.Math.abs;

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
    }

    @Override
    public void loop()
    {
        if (abs(gamepad1.left_stick_y) >= .05)
        {
            double speed = -gamepad2.left_stick_y;
            robot.mastVertical.setPower(speed);
        }

        telemetry.addData("Position", robot.mastVertical.getCurrentPosition());
    }
}
