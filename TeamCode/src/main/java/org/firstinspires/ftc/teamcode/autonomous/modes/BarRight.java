package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBar;

@Disabled
@Autonomous(name="Auto Bar Right")
public class BarRight extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousBar autoBar = new AutonomousBar(this, 1);
    }
}
