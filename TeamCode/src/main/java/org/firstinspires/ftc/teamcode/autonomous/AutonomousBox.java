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
    AutonomousPosition autonomousPosition;;

    public AutonomousBox(LinearOpMode autonomousClass, AutonomousPosition autonomousPosition) throws InterruptedException
    {
        this.autonomousClass = autonomousClass;
        this.autonomousPosition = autonomousPosition;

        robot = new HardwareMecanum();
        robot.init(autonomousClass.hardwareMap);

        base = new Base(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        collector = new Collector(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        mast = new Mast(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2, true);

        mast.resetMastEncoders();

        autonomousClass.waitForStart();

        runOpMode();
    }

    //TODO Move 4050 counts with the back right wheel, move forward 12 inches, crabsteer right, then back up.
    //That will complete the moving of the foundation for the right side.
    public void runOpMode() throws InterruptedException
    {
        base.hookUp();

        base.encoderDriveInches(4, 4, .5, true);
        Thread.sleep(500); //Pause for .5 seconds to ensure full stop.

        base.encoderCrabsteer(0, 22, .5, true);
        Thread.sleep(500);

        base.encoderDriveInches(20, 20, .5, true);
        Thread.sleep(500);

        //Pick skystone
        //Location loc = base.scanStone();
        //this.autonomousClass.telemetry.addData("Location = ", loc.toString());
        //this.autonomousClass.telemetry.update();
        //mast.setMastOnSkystone(loc);

        Thread.sleep(2000);

        base.encoderDriveInches(-20, -20, .5, true);
        Thread.sleep(500); //Fully stop the robot by waiting .5 seconds.

        base.rotateDegreesEncoder(90, .5, true); //90 is always overrotating
        Thread.sleep(500);

        base.encoderDriveInches(110, 110, .5, true);

        base.rotateDegreesEncoder(-90, .5, true);
        Thread.sleep(500);

        base.encoderDriveInches(23.5, 23.5, .2, true);
        Thread.sleep(500);

        base.hookDown();
        Thread.sleep(500);

        base.encoderDriveInches(-35, -35, .5, true);
        Thread.sleep(1000);

        autonomousClass.stop();

        base.hookUp();

        base.rotateDegreesEncoder(-90,.5, true);
        Thread.sleep(500);

        base.encoderDriveInches(44, 44, .5, true);
    }
}