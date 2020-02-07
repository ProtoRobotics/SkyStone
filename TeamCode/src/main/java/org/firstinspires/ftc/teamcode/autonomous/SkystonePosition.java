package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Arm;

// If you look at the skystones as 2 sets of 3 stones, the left and right sets are
// in the same position relative to the BOX and BAR starting positions and
// mirrored on the RIGHT and LEFT sides of the field.

// Based on position of the skystone in the set of 3 stones, this enum gives each
// program the correct positions of the mast, arm, and gripper.

public enum SkystonePosition
{
    LEFT(48500, 223, Arm.GRIPPER_ROTATOR_MAST_LEFT),
    MIDDLE(41000, 0, Arm.GRIPPER_ROTATOR_HORIZONTAL),
    RIGHT(48000, -210, Arm.GRIPPER_ROTATOR_MAST_RIGHT),
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
