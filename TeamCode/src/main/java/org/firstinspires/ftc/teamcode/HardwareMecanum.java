package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public DcMotor leftCollector;
    public DcMotor rightCollector;
    public Servo leftFlapper;
    public Servo rightFlapper;

    //Mast
    public DcMotor mastVertical;
    public DcMotor mastRotator;

    //Arm
    public CRServo armExtender;
    public Servo gripperRotator;
    public Servo leftGripper;
    public Servo rightGripper;

    //Sensors
    public Rev2mDistanceSensor armDistanceSensor;


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
        leftCollector = hwMap.get(DcMotor.class, "leftCollector");
        rightCollector = hwMap.get(DcMotor.class, "rightCollector");
        leftFlapper = hwMap.get(Servo.class, "leftFlapper");
        rightFlapper = hwMap.get(Servo.class, "rightFlapper");

        //Mast
        mastVertical = hwMap.get(DcMotor.class, "mastVertical");
        mastRotator = hwMap.get(DcMotor.class, "mastRotator");

        //Arm
        armExtender = hwMap.get(CRServo.class, "armExtender");
        gripperRotator = hwMap.get(Servo.class, "gripperRotator");
        leftGripper = hwMap.get(Servo.class, "leftGripper");
        rightGripper = hwMap.get(Servo.class, "rightGripper");

        //Sensors
        armDistanceSensor = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "armDistanceSensor");
    }
}
