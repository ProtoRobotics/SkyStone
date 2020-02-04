package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    AutonomousPosition autonomousPosition;

    SkystoneSensor skystoneSensor;
    SkystonePosition skystonePosition;

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

        skystoneSensor = new SkystoneSensor(autonomousClass.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", autonomousClass.hardwareMap.appContext.getPackageName()));

        autonomousClass.waitForStart();

        Thread.sleep(300); //Give the robot time to grab the camera info before moving.
        skystonePosition = SkystonePosition.RIGHT; //TODO remove and replace

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        base.hookUp();

        mast.moveCounts(1100, .3);
        robot.gripperRotator.setPosition(skystonePosition.gripperRotatorPos);
        robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
        robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
        robot.leftFlapper.setPosition(collector.LEFT_FLAPPER_CLOSED);
        robot.rightFlapper.setPosition(collector.RIGHT_FLAPPER_CLOSED);
        robot.mastRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.moveToPosition(skystonePosition.armCounts, 1, false);

        base.encoderDriveInches(7, 7, .3, true);
        Thread.sleep(500); //Pause for .5 seconds to ensure full stop.

        int crabOneDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
        base.encoderCrabsteer(crabOneDirection, 24, .5, true);
        Thread.sleep(500);

        base.encoderDriveInches(20, 20, .4, true);
        Thread.sleep(750);

        robot.mastRotator.setTargetPosition(skystonePosition.mastRotatorCounts);
        robot.mastRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mastRotator.setPower(.5);

        Thread.sleep(700);
        mast.moveCounts(-1100, .3);
        Thread.sleep(1250); //TODO add sequential capability for mast.moveCounts

        robot.leftGripper.setPosition(arm.GRIPPER_LEFT_CLOSED);
        robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_CLOSED);
        Thread.sleep(750);

        mast.moveCounts(400,0.3);

        Thread.sleep(500);

        base.encoderDriveInches(-20, -20, .5, true);

        robot.mastRotator.setTargetPosition(0);
        robot.mastRotator.setPower(.5);
        Thread.sleep(500);

        int rotationThreeDegrees = (autonomousPosition == AutonomousPosition.RIGHT) ? 90 : (-90);
        base.rotateDegreesEncoder(rotationThreeDegrees, .5, true);
        Thread.sleep(600);

        robot.gripperRotator.setPosition(Arm.GRIPPER_ROTATOR_HORIZONTAL);
        arm.moveToPosition(29000, 1, false);

        base.encoderDriveInches(108, 108, .7, true);

        int rotationFourDegrees = (autonomousPosition == autonomousPosition.RIGHT) ? (-90) : 90;
        base.rotateDegreesEncoder(rotationFourDegrees, .5, true);
        Thread.sleep(500);

        arm.moveToPosition(41000, 1, false);
        mast.moveCounts(1000,0.3);

        base.encoderDriveInches(27, 27, .2, true);

        arm.moveToPosition(38000, 1, true);
        
        robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
        robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);

        base.hookDown();
        Thread.sleep(500);

        mast.moveCounts(300,0.5);

        int swivelCounts = -3500;
        double swivelPower = .7;
        if (autonomousPosition == AutonomousPosition.RIGHT)
        {
            robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + swivelCounts);
            robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + swivelCounts);
            robot.rightFront.setPower(swivelPower);
            robot.rightBack.setPower(swivelPower);
        }
        else
        {
            robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() - swivelCounts);
            robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - swivelCounts);
            robot.leftFront.setPower(swivelPower);
            robot.leftBack.setPower(swivelPower);
        }

        boolean swivelBusy = true;
        while (swivelBusy)
        {
            autonomousClass.telemetry.addData("booleans", robot.rightFront.isBusy() + " " + robot.rightBack.isBusy() + " " + robot.leftFront.isBusy() + " " + robot.leftBack.isBusy());
            autonomousClass.telemetry.update();
            Thread.sleep(100);
            swivelBusy = (autonomousPosition == AutonomousPosition.RIGHT) ? (robot.rightFront.isBusy() && robot.rightBack.isBusy()) : (robot.leftFront.isBusy() && robot.leftBack.isBusy());
        }

        base.encoderDriveInches(20,20,0.5,true);

        base.hookUp();

        int crabTwoDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
        base.encoderCrabsteer(crabTwoDirection,15,.7,true);

        mast.moveCounts(-1700,0.5);
        Thread.sleep(500);

        base.encoderDriveInches(-42,-42,.7,true);
        Thread.sleep(500);
    }
}