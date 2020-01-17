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
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2, true);

        mast.resetMastEncoders();

        autonomousClass.waitForStart();

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        {

            mast.moveCounts(1000,.3);
            robot.gripperRotator.setPosition(arm.GRIPPER_ROTATOR_POS_2);
            robot.armExtender.setPower(-1);
            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);
            arm.moveSeconds(4.5,-1);

            int crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
            base.encoderCrabsteer(crabDirection,10,.5,true);

            base.encoderDriveInches(22,22,.3,true);
            Thread.sleep(900);

            int crabOneDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
            base.encoderCrabsteer(crabOneDirection,8,.3,true);
            Thread.sleep(900);


            base.encoderDriveInches(4,4,.2,true);
            Thread.sleep(900);

            mast.moveCounts(-900,.3);
            Thread.sleep(900);

            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_CLOSED);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_CLOSED);
            Thread.sleep(750);

            base.encoderDriveInches(-10,-10,.2,true);//drive up to skystone and pick it up
            Thread.sleep(900);

            int rotationOneDegrees = (autonomousPosition == AutonomousPosition.RIGHT) ? 90 : (-90);
            base.rotateDegreesEncoder(rotationOneDegrees, .3,true);
            Thread.sleep(700);

            base.encoderDriveInches(80,80,.3,true);
            Thread.sleep(900);

            robot.leftGripper.setPosition(arm.GRIPPER_LEFT_OPEN);
            robot.rightGripper.setPosition(arm.GRIPPER_RIGHT_OPEN);

            mast.moveCounts(200,.3);

            base.encoderDriveInches(-15,-15,.5,true);
            Thread.sleep(900);

            mast.moveCounts(-200,.3);

            base.encoderDriveInches(-15,-15,.3,true);
            Thread.sleep(900);

        }



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
    }
}
