package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBox;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;

@Autonomous(name="Auto Box Right")
public class BoxRight extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousBox autoBox = new AutonomousBox(this, AutonomousPosition.RIGHT);
    }
}