package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.Mast;

public class AutonomousBar
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;

    LinearOpMode autonomousClass;
    AutonomousPosition autonomousPosition;

    public AutonomousBar(LinearOpMode autonomousClass, AutonomousPosition autonomousPosition) throws InterruptedException //direction: 0 = left, 1 = right.
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
        {
            //all directional comments based on start position of AutonomousPosition.RIGHT
            //directions would be reversed for start on LEFT side

            int rotationDegrees;
            base.hookUp();
            mast.moveCounts(1100,.3);
            arm.moveSeconds(5.8,-1);
            robot.gripperRotator.setPosition(arm.GRIPPER_ROTATOR_HORIZONTAL);
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
            robot.mastRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //move forward away from wall
            base.encoderDriveInches(27,27,.3,true);
            Thread.sleep(500);

            //crabsteer to the left to align with middle stone in rightmost set of 3
            int crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
            base.encoderCrabsteer(crabDirection,23,.5,true);
            Thread.sleep(500);
            //base.encoderDriveInches(3,3,.3,true);
            Thread.sleep(1500);

            //lower mast to position gripper on top of stone
            mast.moveCounts(-1100,.3);
            Thread.sleep(2000);

            //close grippper
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_CLOSED);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_CLOSED);
            Thread.sleep(500);

            //raise mast to get stone off ground;  reverse robot
            mast.moveCounts(400, .3);
            base.encoderDriveInches(-12,-12,.2,true);//drive up to skystone and pick it up
            Thread.sleep(500);

            //rotate robot 90 degrees to the right
            rotationDegrees = (autonomousPosition == AutonomousPosition.RIGHT) ? 90 : (-90);
            base.rotateDegreesEncoder(rotationDegrees, .4,true);
            Thread.sleep(500);

            //drive across the field to foundation
            base.encoderDriveInches(73,73,.5,true);
            //raise mast to clear the foundation edge with stone
            mast.moveCounts(1000,.3);
            Thread.sleep(500);

            //rotate robot 90 degrees to the left
            arm.moveSeconds(4, -1);
            rotationDegrees = (autonomousPosition == autonomousPosition.RIGHT) ? (-90) : 90;
            base.rotateDegreesEncoder(rotationDegrees, .5, true);
            Thread.sleep(500);

            // drive robot forward to the foundation
            base.encoderDriveInches(11,11,.5,true);
            Thread.sleep(1000);

            //lower mast --> open the gripper
            //mast.moveCounts(-1000,.3);
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
            Thread.sleep(500);
            //reverse robot slightly in order to clear foundation for 90 degree turn to the left
           // base.encoderDriveInches(-4,-4,.3,true);
            //Thread.sleep(900);

            // drive robot backwards away from foundation
            base.encoderDriveInches(-3,-3,.5,true);
            Thread.sleep(500);
            //turn robot 90 degrees to the left in order to drive to center line
            rotationDegrees = (autonomousPosition == autonomousPosition.RIGHT) ? (-90) : 90;
            base.rotateDegreesEncoder(rotationDegrees, .5, true);
            //lower mast to go under bar on return to center line
            mast.moveCounts(-1000,.3);
            Thread.sleep(500);

            //drive robot forward to center line
            base.encoderDriveInches(40,40,.5,true);
            Thread.sleep(1000);
            //crabsteer to the left to move robot against center structure
            crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
            //base.encoderCrabsteer(crabDirection,4,4,true);

        }
    }
}
