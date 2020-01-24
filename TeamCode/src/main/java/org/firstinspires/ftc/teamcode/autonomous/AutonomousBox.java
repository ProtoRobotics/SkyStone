package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.ImuRotator;
import org.firstinspires.ftc.teamcode.Mast;
import org.firstinspires.ftc.teamcode.Location; //JAD 12/3/19

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


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
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);

        mast.resetMastEncoders();

        autonomousClass.waitForStart();

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        base.hookUp();

        mast.moveCounts(1000, .3);
        robot.gripperRotator.setPosition(arm.GRIPPER_ROTATOR_POS_2);
        robot.armExtender.setPower(-1);
        robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
        robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
        arm.moveSeconds(4.5, -1);

        base.encoderDriveInches(4, 4, .3, true);
        Thread.sleep(500); //Pause for .5 seconds to ensure full stop.

        int crabOneDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
        base.encoderCrabsteer(crabOneDirection, 24, .5, true);
        Thread.sleep(500);

        base.encoderDriveInches(20, 20, .5, true);
        Thread.sleep(750);

        mast.moveCounts(-1100, .3);
        Thread.sleep(750); //TODO add sequential capability for mast.moveCounts

        robot.leftGripper.setPosition(arm.GRIPPER_LEFT_CLOSED);
        robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_CLOSED);
        Thread.sleep(750);

        mast.moveCounts(200,0.3);

        Thread.sleep(2000);

        base.encoderDriveInches(-20, -20, .5, true);
        Thread.sleep(500); //Fully stop the robot by waiting .5 seconds.

        int rotationThreeDegrees = (autonomousPosition == AutonomousPosition.RIGHT) ? 90 : (-90);
        base.rotateDegreesEncoder(rotationThreeDegrees, .5, true);
        Thread.sleep(500);

        arm.moveSeconds(2,1);

        base.encoderDriveInches(111, 111, .5, true);
        Thread.sleep(500);

        int rotationFourDegrees = (autonomousPosition == autonomousPosition.RIGHT) ? (-90) : 90;
        base.rotateDegreesEncoder(rotationFourDegrees, .5, true);
        Thread.sleep(500);

        arm.moveSeconds(2,-1);
        mast.moveCounts(1000,0.3);

        base.encoderDriveInches(29.5, 29.5, .2, true);
        Thread.sleep(500);

        base.hookDown();
        Thread.sleep(500);


        int swivelCounts = -3500;
        if (autonomousPosition == AutonomousPosition.RIGHT)
        {
            robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + swivelCounts);
            robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + swivelCounts);
            robot.rightFront.setPower(.3);
            robot.rightBack.setPower(.3);
        }
        else
        {
            robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() - swivelCounts);
            robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - swivelCounts);
            robot.leftFront.setPower(.5);
            robot.leftBack.setPower(.5);
        }

        boolean swivelBusy = true;
        while (swivelBusy)
        {
            autonomousClass.telemetry.addData("booleans", robot.rightFront.isBusy() + " " + robot.rightBack.isBusy() + " " + robot.leftFront.isBusy() + " " + robot.leftBack.isBusy());
            autonomousClass.telemetry.update();
            Thread.sleep(100);
            swivelBusy = (autonomousPosition == AutonomousPosition.RIGHT) ? (robot.rightFront.isBusy() && robot.rightBack.isBusy()) : (robot.leftFront.isBusy() && robot.leftBack.isBusy());
        }

        autonomousClass.telemetry.addData("Out of the loop", "yeah");
        autonomousClass.telemetry.update();

        Thread.sleep(1000);

        base.encoderDriveInches(20,20,0.4,true);

        robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
        robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);

        base.hookUp();

        int crabTwoDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
        base.encoderCrabsteer(crabTwoDirection,16,.5,true);
        Thread.sleep(500);

        base.encoderDriveInches(-42,-42,.5,true);
        Thread.sleep(500);
    }
}