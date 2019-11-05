package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ImuRotateTesttt")
public class ImuRotateTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        HardwareMecanum robot = new HardwareMecanum();
        robot.init(hardwareMap);

        ImuRotator imuRotator = new ImuRotator(robot);
        imuRotator.rotateIMU(.5, 90);
    }
}
