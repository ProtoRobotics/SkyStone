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

            base.encoderDriveInches(22,22,.5,true);
            Thread.sleep(900);

            int crabOneDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
            base.encoderCrabsteer(crabOneDirection,18,.5,true);
            Thread.sleep(900);

            mast.moveCounts(1000,.3);
            robot.gripperRotator.setPosition(arm.GRIPPER_ROTATOR_POS_2);
            robot.armExtender.setPower(-1);
            robot.leftGripper.setPosition((arm.GRIPPER_LEFT_OPEN));
            arm.moveSeconds(5,-1);

            base.encoderDriveInches(4,4,.2,true);
            Thread.sleep(900);

            base.encoderDriveInches(-4,-4,.2,true);//drive up to skystone and pick it up
            Thread.sleep(900);

            int crabTwoDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
            base.encoderCrabsteer(crabTwoDirection,76,.5,true);
            Thread.sleep(900);

            base.encoderDriveInches(6,6,.1,true);
            Thread.sleep(900);

            base.encoderDriveInches(-5,-5,.1,true);//drive up to the foundations and set block down
            Thread.sleep(900);

            int crabThreeDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 1 : 0;
            base.encoderCrabsteer(crabThreeDirection,45,.5,true);//park on the line
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
    }
}
