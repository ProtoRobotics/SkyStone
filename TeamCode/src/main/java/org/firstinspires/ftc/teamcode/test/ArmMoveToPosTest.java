package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Autonomous(name = "ArmMoveToPosTest")
public class ArmMoveToPosTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        HardwareMecanum robot = new HardwareMecanum();
        robot.init(hardwareMap);

        Arm arm = new Arm(this, robot, gamepad1, gamepad2, true);

        //arm.moveToPos(36);
    }
}
