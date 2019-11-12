package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.Mast;

public class AutonomousBox
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;

    LinearOpMode autonomousClass;
    int direction;

    public AutonomousBox(LinearOpMode autonomousClass, int direction) throws InterruptedException
    {
        this.autonomousClass = autonomousClass;
        this.direction = direction;

        robot = new HardwareMecanum();
        robot.init(autonomousClass.hardwareMap);

        base = new Base(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        collector = new Collector(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        mast = new Mast(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        base.encoderCrabsteer(-22, .7, 0);
        base.encoderDriveInches(20, 20, .7, true);

        //Pick skystone
        Thread.sleep(2000);

        base.encoderDriveInches(-24, -24, .7, true);
        Thread.sleep(300); //Fully stop the robot by waiting .3 seconds.

        base.rotateDegreesEncoder(90, .4, true);

        base.encoderDriveInches(132, 132, 1, true);

        base.rotateDegreesEncoder(-90, .4, true);
        base.encoderDriveInches(24, 24, .7, true);

        //lower hook

        base.encoderDriveInches(-22, 22, 1, true);

        //raise hook

        base.rotateDegreesEncoder(-90, .4, true);
        base.encoderDriveInches(22, 22, .7, true);
    }
}