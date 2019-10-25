package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.Mast;

@Autonomous(name="Autonomous Box", group="Auto")
public class AutonomousBox extends LinearOpMode
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);

        base = new Base(this, robot, gamepad1, gamepad2);
        collector = new Collector(this, robot, gamepad1, gamepad2);
        mast = new Mast(this, robot, gamepad1, gamepad2);
        arm = new Arm(this, robot, gamepad1, gamepad2);

        base.crabsteer(-22, .7);
        base.encoderDriveInches(20, 20, .7, true);

        //Pick skystone
        Thread.sleep(2000);

        base.encoderDriveInches(-24, -24, .7, true);
        Thread.sleep(300); //Fully stop the robot by waiting .3 seconds.

        base.rotateIMU(90);

        base.encoderDriveInches(132, 132, 1, true);

        base.rotateIMU(-90);
        base.encoderDriveInches(24, 24, .7, true);

        //lower hook

        base.encoderDriveInches(-22, 22, 1, true);

        //raise hook

        base.rotateIMU(-90);
        base.encoderDriveInches(22, 22, .7, true);
    }
}