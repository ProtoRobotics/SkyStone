package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBox;

@Disabled
@Autonomous(name="Auto Box Left")
public class BoxLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousBox autoBox = new AutonomousBox(this, 0);
    }
}
