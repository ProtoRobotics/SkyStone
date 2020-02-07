package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousPosition;
import org.firstinspires.ftc.teamcode.autonomous.SkystonePosition;
import org.firstinspires.ftc.teamcode.autonomous.SkystoneSensor;

@TeleOp(name="SkystoneSensorTest")
public class SkystoneSensorTest extends OpMode
{
    SkystoneSensor skystoneSensor;



    @Override
    public void init() {
        skystoneSensor = new SkystoneSensor(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()), AutonomousPosition.LEFT);
    }

    @Override
    public void loop() {

        if (skystoneSensor.getSkystonePosition() == SkystonePosition.LEFT)
        {
            telemetry.addData("Position:", " Left");
            telemetry.update();
        }
        else if (skystoneSensor.getSkystonePosition() == SkystonePosition.MIDDLE)
        {
            telemetry.addData("Position: ", "Middle");
            telemetry.update();
        }
        else if (skystoneSensor.getSkystonePosition() == SkystonePosition.RIGHT)
        {
            telemetry.addData("Position: ", "Right");
            telemetry.update();
        }
        else if (skystoneSensor.getSkystonePosition() == SkystonePosition.UNKNOWN)
        {
            telemetry.addData("Position: ", "UNKNOWN");
            telemetry.update();
        }
    }



}
