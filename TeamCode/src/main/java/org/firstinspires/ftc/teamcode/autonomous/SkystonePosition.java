package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Arm;

public enum SkystonePosition
{
    LEFT(49000, 240, Arm.GRIPPER_ROTATOR_MAST_LEFT),
    MIDDLE(41000, 0, Arm.GRIPPER_ROTATOR_HORIZONTAL),
    RIGHT(49000, -240, Arm.GRIPPER_ROTATOR_MAST_RIGHT),
    UNKNOWN(41000, 0, Arm.GRIPPER_ROTATOR_HORIZONTAL);

    public int armCounts;
    public int mastRotatorCounts;
    public double gripperRotatorPos;

    private SkystonePosition(int armCounts, int mastRotatorCounts, double gripperRotatorPos)
    {
        this.armCounts = armCounts;
        this.mastRotatorCounts = mastRotatorCounts;
        this.gripperRotatorPos = gripperRotatorPos;
    }
}
