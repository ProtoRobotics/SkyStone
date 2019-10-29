package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBar;

@Autonomous(name="Auto Bar Left")
public class BarLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousBar autoBar = new AutonomousBar(this, 0);
    }
}
