package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.ImuRotator;
import org.firstinspires.ftc.teamcode.Mast;

public class AutonomousBox
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;
    ImuRotator imuRotator;

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

        imuRotator = new ImuRotator(autonomousClass, robot);

        autonomousClass.waitForStart();

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        base.encoderDriveInches(4, 4, .7, true);
        Thread.sleep(500); //Pause for .5 seconds to ensure full stop.

        base.encoderCrabsteer(0, 22, .5, true);
        Thread.sleep(500);

        base.encoderDriveInches(20, 20, .7, true);

        //Pick skystone
        Thread.sleep(2000);

        base.encoderDriveInches(-23, -23, .7, true);
        Thread.sleep(500); //Fully stop the robot by waiting .3 seconds.

        imuRotator.rotateIMU(.5, 90);
        Thread.sleep(500);

        base.encoderDriveInches(100, 100, 1, true);

        imuRotator.rotateIMU(.5, -90);
        Thread.sleep(500);

        base.encoderDriveInches(24, 24, .7, true);
        Thread.sleep(500);

        base.hookDown();
        Thread.sleep(500);

        base.encoderDriveInches(-22, 22, 1, true);
        Thread.sleep(500);

        base.hookUp();

        imuRotator.rotateIMU(.5, -90);
        Thread.sleep(500);

        base.encoderDriveInches(22, 22, .7, true);
    }
}