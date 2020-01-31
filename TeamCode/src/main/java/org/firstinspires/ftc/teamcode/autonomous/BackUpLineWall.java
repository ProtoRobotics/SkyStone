package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Base;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.ImuRotator;
import org.firstinspires.ftc.teamcode.Mast;
import org.firstinspires.ftc.teamcode.Location; //JAD 12/3/19
import org.firstinspires.ftc.teamcode.autonomous.modes.BackupLineCloseLeft;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class BackUpLineWall {
    Base base;
    Collector collector;
    Mast mast;
    Arm arm;

    HardwareMecanum robot;

    LinearOpMode autonomousClass;
    AutonomousPosition autonomousPosition;

    public BackUpLineWall(LinearOpMode autonomousClass, AutonomousPosition autonomousPosition) throws InterruptedException {
        this.autonomousClass = autonomousClass;
        this.autonomousPosition = autonomousPosition;

        robot = new HardwareMecanum();
        robot.init(autonomousClass.hardwareMap);

        base = new Base(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        collector = new Collector(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        mast = new Mast(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);
        arm = new Arm(autonomousClass, robot, autonomousClass.gamepad1, autonomousClass.gamepad2);

        mast.resetMastEncoders();

        autonomousClass.waitForStart();

        runOpMode();
    }






    public void runOpMode()throws InterruptedException
    {
        int crabDirection = (autonomousPosition == AutonomousPosition.RIGHT) ? 0 : 1;
        base.encoderCrabsteer(crabDirection,12,.3,true);
    }
}
