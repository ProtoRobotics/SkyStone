package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Disabled
@Autonomous(name="TestRotateEncoder")
public class TestRotateEncoder extends LinearOpMode
{
    //Used to find counts per rotation/counts per degree
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        HardwareMecanum robot = new HardwareMecanum();
        robot.init(hardwareMap);
        Base base = new Base(this, robot, gamepad1, gamepad2);

        base.rotateDegreesEncoder(90, .5, true);

        Thread.sleep(500);

        base.rotateDegreesEncoder(-90, .5, true);

        Thread.sleep(500);

        base.rotateDegreesEncoder(360, .5, true);
    }
}
