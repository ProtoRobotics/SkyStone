package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBox;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;
import org.firstinspires.ftc.teamcode.autonomous.BackUpLineWall;

@Autonomous(name="BackupLineCloseLeft")
public class BackupLineWallLeft extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        BackUpLineWall Backup = new BackUpLineWall(this, AutonomousPosition.LEFT);
    }
}
