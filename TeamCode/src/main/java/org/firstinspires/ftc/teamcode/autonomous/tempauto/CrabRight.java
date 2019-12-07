package org.firstinspires.ftc.teamcode.autonomous.tempauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Autonomous(name="Crab Right")
public class CrabRight extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        HardwareMecanum robot = new HardwareMecanum();
        robot.init(hardwareMap);
        Base base = new Base(this, robot, gamepad1, gamepad2);

        base.encoderCrabsteer(1, 16, .5, true);
    }
}
