package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBox;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;
import org.firstinspires.ftc.teamcode.autonomous.BackUpLineWall;

@Autonomous(name="BackupLineCloseRight")
public class BackupLineWallRight extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        BackUpLineWall Backup = new BackUpLineWall(this, AutonomousPosition.RIGHT);
    }
}
