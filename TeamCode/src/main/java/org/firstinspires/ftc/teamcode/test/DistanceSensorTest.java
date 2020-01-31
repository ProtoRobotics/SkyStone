package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

@Disabled
@Autonomous(name = "distanceSensorTest")
public class DistanceSensorTest extends OpMode {

    HardwareMecanum robot;

    public void init()
    {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);
        robot.armDistanceSensor.initialize();
    }

    public void loop()
    {
        double distance = robot.armDistanceSensor.getDistance(DistanceUnit.CM);

        telemetry.addData("Distance (M)", distance);
        telemetry.update();

        if (gamepad1.x)
        {
            robot.armExtender.setPower(-1);
        }
        else if (gamepad1.b)
        {
            robot.armExtender.setPower(1);
        }
        else
        {
            robot.armExtender.setPower(0);
        }
    }

}
