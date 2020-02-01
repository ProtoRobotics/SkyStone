package org.firstinspires.ftc.teamcode.autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;

@Disabled
@Autonomous(name="Old Auto Box Left")
public class OldBoxLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        OldAutonomousBox autoBox = new OldAutonomousBox(this, AutonomousPosition.LEFT);
    }
}
