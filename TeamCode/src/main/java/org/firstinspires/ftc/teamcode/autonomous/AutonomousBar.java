package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.Mast;

import static org.firstinspires.ftc.teamcode.Arm.GRIPPER_ROTATOR_VERTICAL;

public class AutonomousBar
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;

    LinearOpMode autonomousClass;
    AutonomousPosition autonomousPosition;

    SkystoneSensor skystoneSensor;
    SkystonePosition skystonePosition;

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
        skystoneSensor = new SkystoneSensor(autonomousClass.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", autonomousClass.hardwareMap.appContext.getPackageName()),autonomousPosition);

        autonomousClass.waitForStart();

        Thread.sleep(300); //Give the robot time to grab the camera info before moving.
        skystonePosition = SkystonePosition.MIDDLE; //TODO remove and replace

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
            //arm.moveSeconds(5.8,-1);
            arm.moveToPosition(skystonePosition.armCounts,1,false);
            robot.gripperRotator.setPosition(skystonePosition.gripperRotatorPos);
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
            robot.leftFlapper.setPosition(collector.LEFT_FLAPPER_CLOSED);
            robot.rightFlapper.setPosition(collector.RIGHT_FLAPPER_CLOSED);
            robot.mastRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //move forward away from wall
            base.encoderDriveInches(27,27,.3,true);
            Thread.sleep(500);

            //crabsteer to the left to align with middle stone in rightmost set of 3
            int crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
            base.encoderCrabsteer(crabDirection,23,.5,true);

            robot.mastRotator.setTargetPosition(skystonePosition.mastRotatorCounts);
            robot.mastRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mastRotator.setPower(.5);

            Thread.sleep(2000);

            //lower mast to position gripper on top of stone
            mast.moveCounts(-1100,.3);

            Thread.sleep(2000);

            //close grippper
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_CLOSED);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_CLOSED);
            Thread.sleep(500);

            //raise mast to get stone off ground;  reverse robot
            mast.moveCounts(400, .3);
            //base.encoderDriveInches(-5,-5,.2,true);//drive up to skystone and pick it up
            robot.mastRotator.setTargetPosition(0);
            robot.mastRotator.setPower(.5);

            Thread.sleep(1000);

            //rotate robot 90 degrees to the right
            rotationDegrees = (autonomousPosition == AutonomousPosition.RIGHT) ? 90 : (-90);
            base.rotateDegreesEncoder(rotationDegrees, .4,true);
            Thread.sleep(500);

            //drive across the field to foundation
            base.encoderDriveInches(56,56,.5,true);
            //raise mast to clear the foundation edge with stone
            mast.moveCounts(1000,.3);
            Thread.sleep(3000);  //TODO reduce back down to 500
            //rotate gripper to vertical position because robot will be approaching foundation from side
            robot.gripperRotator.setPosition(GRIPPER_ROTATOR_VERTICAL);

            //for consistency, always move arm to the MIDDLE position count
            skystonePosition = SkystonePosition.MIDDLE;
            arm.moveToPosition(45000, 1,false);

            crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
            base.encoderCrabsteer(crabDirection,24,.3,true);
            Thread.sleep(1500);

            //open the gripper to drop stone
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
            Thread.sleep(500);

            crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
            base.encoderCrabsteer(crabDirection,20,.3,true);
            mast.moveCounts(-1000, .3);
            Thread.sleep(1000); //TODO - reduce to 500

            //drive robot backwardsto center line
            base.encoderDriveInches(-12,-12,.3,true);
            //Thread.sleep(1000);
            //crabsteer to the left to move robot against center structure
            //crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
            //base.encoderCrabsteer(crabDirection,4,4,true);

        }
    }
}
