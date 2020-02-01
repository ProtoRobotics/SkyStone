package org.firstinspires.ftc.teamcode.autonomous.dummyauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;

@Disabled
@Autonomous(name="BackupLineCenterLeft")
public class BackupLineCenterLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        BackupLineCenter autoBox = new BackupLineCenter(this, AutonomousPosition.LEFT);
    }
}
