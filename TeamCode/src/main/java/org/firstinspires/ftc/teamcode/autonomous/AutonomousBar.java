package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
            arm.moveSeconds(5,-1);
            robot.gripperRotator.setPosition(arm.GRIPPER_ROTATOR_POS_2);
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);

            //move forward away from wall
            base.encoderDriveInches(26,26,.3,true);
            Thread.sleep(500);

            //crabsteer to the left to align with middle stone in rightmost set of 3
            int crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
            base.encoderCrabsteer(crabDirection,12.5,.5,true);
            Thread.sleep(2000);

            //extend arm and raise mast to position gripper above stone
            mast.moveCounts(-1100,.3);
            Thread.sleep(1000);

            //close grippper
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_CLOSED);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_CLOSED);
            Thread.sleep(500);

            //raise mast to get stone off ground;  reverse robot
            mast.moveCounts(150, .3);
            base.encoderDriveInches(-10,-10,.2,true);//drive up to skystone and pick it up
            Thread.sleep(900);

            //rotate robot 90 degrees to the right
            rotationDegrees = (autonomousPosition == AutonomousPosition.RIGHT) ? 90 : (-90);
            base.rotateDegreesEncoder(rotationDegrees, .3,true);
            Thread.sleep(700);

            //drive across the field to foundation
            base.encoderDriveInches(70,70,.3,true);
            Thread.sleep(900);

            //rotate robot 90 degrees to the left
            rotationDegrees = (autonomousPosition == autonomousPosition.RIGHT) ? (-90) : 90;
            base.rotateDegreesEncoder(rotationDegrees, .5, true);
            Thread.sleep(500);

            //raise mast to clear the foundation edge with stone
            mast.moveCounts(1000,.3);
            Thread.sleep(1000);

            // drive robot forward to the foundation
            base.encoderDriveInches(15,15,.5,true);
            Thread.sleep(900);

            //lower mast --> open the gripper
            //mast.moveCounts(-1000,.3);
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);

            //reverse robot slightly in order to clear foundation for 90 degree turn to the left
            base.encoderDriveInches(-4,-4,.3,true);
            Thread.sleep(900);

            //lower mast to go under bar on return to center line
            mast.moveCounts(-1000,.3);

            //turn robot 90 degrees to the left in order to drive to center line
            rotationDegrees = (autonomousPosition == autonomousPosition.RIGHT) ? (-90) : 90;
            base.rotateDegreesEncoder(rotationDegrees, .5, true);
            Thread.sleep(500);

            //drive robot forward to center line
            base.encoderDriveInches(48,48,.5,true);
        }
    }
}
