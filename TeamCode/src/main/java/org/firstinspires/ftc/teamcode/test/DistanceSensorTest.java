package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Autonomous(name = "distanceSensorTest")

public class distanceSensorTest extends OpMode {

    HardwareMecanum robot;

    public void init()
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);
        robot.mastDistanceSensor.initialize();
    }

    public void loop()
    {
        double distance = robot.mastDistanceSensor.getDistance(DistanceUnit.CM);

        telemetry.addData("Distance (M)", distance);
        telemetry.update();

        if (distance < 10)
        {
            robot.leftCollector.setPower(1);
        }
        else
        {
            robot.leftCollector.setPower(0);
        }
    }

}
