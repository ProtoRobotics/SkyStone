package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class HardwareMecanum
{
    //Base
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor rightFront;
    public BNO055IMU imu;
    public Servo hook;

    //Collector
    public CRServo leftCollector;
    public CRServo rightCollector;

    //Mast
    public DcMotor mastVertical;
    public DcMotor mastRotator;

    //Arm
    public CRServo armExtender;
    public Servo gripperRotator;
    public Servo gripper;

    //Sensors
    public NormalizedColorSensor rightColorSensor;
    public NormalizedColorSensor leftColorSensor;
    public Rev2mDistanceSensor baseDistanceSensor;
    public Rev2mDistanceSensor armDistanceSensor;
    public Rev2mDistanceSensor mastDistanceSensor;


    public HardwareMap hwMap;

    public void init(HardwareMap hwMap)
    {
        this.hwMap = hwMap;

        //Base
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        imu = hwMap.get(BNO055IMU.class, "imu");
        hook = hwMap.get(Servo.class, "hook");

        //Collector
        leftCollector = hwMap.get(CRServo.class, "leftCollector");
        rightCollector = hwMap.get(CRServo.class, "rightCollector");

        //Mast
        mastVertical = hwMap.get(DcMotor.class, "mastVertical");
        mastRotator = hwMap.get(DcMotor.class, "mastRotator");

        //Arm
        armExtender = hwMap.get(CRServo.class, "armExtender");
        gripperRotator = hwMap.get(Servo.class, "gripperRotator");
        gripper = hwMap.get(Servo.class, "gripper");

        //Sensors
        leftColorSensor = hwMap.get(NormalizedColorSensor.class,"leftColorSensor");
        rightColorSensor = hwMap.get(NormalizedColorSensor.class,"rightColorSensor");
        baseDistanceSensor = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "baseDistanceSensor");
        armDistanceSensor = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "armDistanceSensor");
        mastDistanceSensor = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "mastDistanceSensor");
    }
}
