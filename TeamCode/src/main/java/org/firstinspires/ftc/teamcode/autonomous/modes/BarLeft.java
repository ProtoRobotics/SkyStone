package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousBar;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;

@Autonomous(name="Auto Bar Left")
public class BarLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousBar autoBar = new AutonomousBar(this, AutonomousPosition.LEFT);
        HardwareMecanum robot = new HardwareMecanum();
        robot.init(hardwareMap);
        Base base = new Base(this, robot, gamepad1, gamepad2);
        base.encoderDriveInches(22,22,.5,true);
        Thread.sleep(900);
        base.encoderCrabsteer(1,22,.5,true);
        Thread.sleep(900);
        base.encoderDriveInches(5,5,.2,true);
        Thread.sleep(900);
        base.encoderDriveInches(-5,-5,.2,true);
        Thread.sleep(900);
        base.encoderCrabsteer(-1,80,.5,true);
        Thread.sleep(900);
        base.encoderDriveInches(6,6,.1,true);
        Thread.sleep(900);
        base.encoderDriveInches(-5,-5,.1,true);
        Thread.sleep(900);
        base.encoderCrabsteer(1,45,.5,true);
    }
}
