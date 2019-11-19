package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
    int direction;

    public AutonomousBar(LinearOpMode autonomousClass, int direction) throws InterruptedException //direction: 0 = left, 1 = right.
    {
        this.autonomousClass = autonomousClass;
        this.direction = direction;

        robot = new HardwareMecanum();
        robot.init(autonomousClass.hardwareMap);

        base = new Base(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        collector = new Collector(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        mast = new Mast(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);

        autonomousClass.waitForStart();

        runOpMode();
    }

    public void runOpMode() throws InterruptedException
    {
        if(direction == 0)
        {
            base.encoderCrabsteer(-20,.7,0, true);
        }
        else if(direction == 1)
        {
            base.encoderCrabsteer(20,.7,0, true);
        }

        //base.crabsteer(-23, .7, true);

        //Thread.sleep(2000);
        //base.rotateDegreesEncoder(90, .4, true);
        //rotate robot and pick up skystone
        //base.encoderDriveInches(74,74,.7,true);

        //Thread.sleep(2000);
        //put skystone on base
        //base.encoderDriveInches(-61,-61,.7,true);
    }
}
