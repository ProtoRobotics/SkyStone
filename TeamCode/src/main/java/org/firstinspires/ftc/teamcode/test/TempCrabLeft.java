package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Autonomous(name="TempCrabLeft")
public class TempCrabLeft extends LinearOpMode
{
    HardwareMecanum robot;
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);

        waitForStart();

        Base base = new Base(this, robot, gamepad1, gamepad2);

        base.encoderCrabsteer(1, 24, .5, true);

        robot.leftCollector.setPower(1);

        sleep(3000);
    }
}
