package org.firstinspires.ftc.teamcode.autonomous.tempauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

import static org.firstinspires.ftc.teamcode.Base.HOOK_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.Base.HOOK_UP_POSITION;

@Disabled
@Autonomous(name="Crab Left")
public class CrabLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        HardwareMecanum robot = new HardwareMecanum();
        robot.init(hardwareMap);
        Base base = new Base(this, robot, gamepad1, gamepad2);

        base.encoderCrabsteer(0, 16, .5, true);

    }
}
