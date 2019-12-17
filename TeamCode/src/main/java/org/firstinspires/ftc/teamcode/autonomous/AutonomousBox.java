package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.ImuRotator;
import org.firstinspires.ftc.teamcode.Mast;
import org.firstinspires.ftc.teamcode.Location; //JAD 12/3/19


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
        mast = new Mast(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2, true);
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2, true);

        mast.resetMastEncoders();

        imuRotator = new ImuRotator(autonomousClass, robot);

        autonomousClass.waitForStart();

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        base.hookUp();

        base.encoderDriveInches(4, 4, .7, true);
        Thread.sleep(500); //Pause for .5 seconds to ensure full stop.

        base.encoderCrabsteer(0, 22, .5, true);
        Thread.sleep(500);

        base.encoderDriveInches(20, 20, .7, true);
        Thread.sleep(500);  //JAD 12/3/19

        //Pick skystone
        Location loc = base.scanStone();
        this.autonomousClass.telemetry.addData("Location = ", loc.toString());
        this.autonomousClass.telemetry.update();
        mast.setMastOnSkystone(loc);

        Thread.sleep(2000);

        base.encoderDriveInches(-20, -20, .7, true);
        Thread.sleep(500); //Fully stop the robot by waiting .5 seconds.

        imuRotator.rotateIMU(.5, 83); //90 is always overrotating
        Thread.sleep(500);

        base.encoderDriveInches(110, 110, .7, true);

        imuRotator.rotateIMU(.5, -85);
        Thread.sleep(500);

        base.encoderDriveInches(23.5, 23.5, .2, true);
        Thread.sleep(500);

        base.hookDown();
        Thread.sleep(500);

        base.encoderDriveInches(-35, -35, .5, true);
        Thread.sleep(1000);

        autonomousClass.stop();

        base.hookUp();

        imuRotator.rotateIMU(.3, -85);
        Thread.sleep(500);

        base.encoderDriveInches(44, 44, .7, true);
    }
}