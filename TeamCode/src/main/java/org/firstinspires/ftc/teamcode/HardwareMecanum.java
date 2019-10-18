package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    //Collector
    public CRServo leftCollector;
    public CRServo rightCollector;

    //Mast
    public DcMotor mastVertical;
    public DcMotor mastRotator;

    //Arm
    public CRServo armExtender;
    public Servo gripperRotater;
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

        //COLLECTOR
        leftCollector = hwMap.get(CRServo.class, "leftCollector");
        rightCollector = hwMap.get(CRServo.class, "rightCollector");

        //MAST
        mastVertical = hwMap.get(DcMotor.class, "mastVertical");
        mastRotator = hwMap.get(DcMotor.class, "mastRotator");

        //ARM
        armExtender = hwMap.get(CRServo.class, "armExtender");
        gripperRotater = hwMap.get(Servo.class, "gripperRotater");
        gripper = hwMap.get(Servo.class, "gripper");

    }
}
