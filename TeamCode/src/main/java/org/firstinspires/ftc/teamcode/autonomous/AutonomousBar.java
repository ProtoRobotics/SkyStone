package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.Mast;

@Autonomous(name="Autonomous Bar", group="Auto")
public class AutonomousBar extends LinearOpMode
{
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);

        base = new Base(this, robot, gamepad1, gamepad2);
        collector = new Collector(this, robot, gamepad1, gamepad2);
        mast = new Mast(this, robot, gamepad1, gamepad2);
        arm = new Arm(this, robot, gamepad1, gamepad2);

        base.encoderDriveInches(21,21,.7,true);
        base.crabsteer(-23, .7);

        Thread.sleep(2000);
        base.rotateIMU(90);
        //rotate robot and pick up skystone
        base.encoderDriveInches(74,74,.7,true);

        Thread.sleep(2000);
        //put skystone on base
        base.encoderDriveInches(-61,-61,.7,true);

    }
}
