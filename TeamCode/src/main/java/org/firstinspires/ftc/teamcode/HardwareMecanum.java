package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

public class HardwareMecanum
{
    //Base
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor rightFront;
    public BNO055IMU imu;
    public Servo hook;
    public NormalizedColorSensor sensorColorRight;
    public NormalizedColorSensor sensorColorLeft;
    public Rev2mDistanceSensor baseDistSensor;

    //Collector
    public CRServo leftCollector;
    public CRServo rightCollector;

    //Mast
    public DcMotor mastVertical;
    public DcMotor mastRotator;

    //Arm
    public Rev2mDistanceSensor armDistSensor;
    public Servo armExtender;
    public Servo gripperRotator;
    public Servo gripper;

    public HardwareMap hwMap;

    public void init(HardwareMap hwMap)
    {
        //BASE
        this.hwMap = hwMap;
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        imu = hwMap.get(BNO055IMU.class, "imu");
        hook = hwMap.get(Servo.class, "hook");
        sensorColorLeft = hwMap.get(NormalizedColorSensor.class,"sensorColorLeft");
        sensorColorRight = hwMap.get(NormalizedColorSensor.class,"sensorColorRight");
        //baseDistSensor = hwMap.get(Rev2mDistanceSensor.class, "bastDistSensor");

        //COLLECTOR
        //leftCollector = hwMap.get(CRServo.class, "leftCollector");
        //rightCollector = hwMap.get(CRServo.class, "rightCollector");

        //MAST
        //mastVertical = hwMap.get(DcMotor.class, "mastVertical");
        //mastRotator = hwMap.get(DcMotor.class, "mastRotator");

        //ARM
        armDistSensor = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "armDistSensor");
        //armExtender = hwMap.get(Servo.class, "armExtender");
        //gripperRotator = hwMap.get(Servo.class, "gripperRotator");
        //gripper = hwMap.get(Servo.class, "gripper");
    }
}
