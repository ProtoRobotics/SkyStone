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

    }
}
