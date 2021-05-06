package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.data.MyScalar;
import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.firstinspires.ftc.teamcode.robot_components.PowerLauncher;

public class TowerGoal extends CVObject {

    public TowerGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("tower", pipeline, xPID, wPID);
        lowerHSV = LOWER_BLUE_TOWER_HSV;
        upperHSV = UPPER_BLUE_TOWER_HSV;
        cover = 0;
    }

    public int getLeftRightError(int offSet) {
        return x - (320 + offSet - x - w);
    }

    // Tests to make sure there is the slot for the high goal
    // Makes the object detection pretty bulletproof
    private boolean hasHatShape(int x, int y, int w, int h) {
        int row = y + (int) (0.8 * h); // 80% of the way down from top of blue tower goal

        // Left flap has to be blue
        MyScalar scalar = findHSV(row, x + (int) (0.1 * w));
        if (!scalar.inRange(LOWER_BLUE_TOWER_HSV, UPPER_BLUE_TOWER_HSV)) {
            return false;
        }

        // Right flap has to be blue
        scalar = findHSV(row, x + (int) (0.9 * w));
        if (!scalar.inRange(LOWER_BLUE_TOWER_HSV, UPPER_BLUE_TOWER_HSV)) {
            return false;
        }

        // Middle part shouldn't be blue (since that's the slot for the rings to enter the high goal)
        scalar = findHSV(row, x + (int) (0.5 * w));
        if (scalar.inRange(LOWER_BLUE_TOWER_HSV, UPPER_BLUE_TOWER_HSV)) {
            return false;
        }
        return true;
    }

    // Testing to make sure the detected object is the tower goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        double r = 1.0 * w / h; // ratio is usually about 1.5
        // width 34 is back of the field, closest is 150
        return (30 < w && w < 130 && 18 < h && h < 65 && r > 1.3) /* && hasHatShape(x, y, w, h)*/;
    }
    /*
        Some values for reference:
        w       h
        34      22
        52      34
        77      49
        110     45 (top of goal is out of view)
     */

    public boolean onTarget() {
        return getErrorX() == 0 && getErrorW() == 0;
    }

    public boolean targetInRange(int radius) {
        return getAbsErrorX() <= radius && getAbsErrorW() <= radius;
    }

    public double cvtH2VerticalDist() {
        return -0.0000270563 * Math.pow(h, 5) + 0.00422078 * Math.pow(h, 4)
                - 0.260795 * Math.pow(h, 3) + 7.98312 * Math.pow(h, 2) - 121.456 * h + 747;
    }

    public double cvtFt2LaunchOffset(double ft) {
        double val = -0.557143 * Math.pow(ft, 5) + 21.2024 * Math.pow(ft, 4)
                - 319.119 * Math.pow(ft, 3) + 2377.3 * Math.pow(ft, 2) - 8789.82 * ft + 12939.5;
        return val / 1000.0;
    }

    public double findLaunchAngle(double angle) {
        double dist = cvtH2VerticalDist() / Math.cos(Math.toRadians(angle));
        return PowerLauncher.PERFECT_LAUNCH_ANGLE + cvtFt2LaunchOffset(dist);
    }
    /*
        Some more values for reference:
        h           distance (ft)       launch offset (rel. to perfect)
        38          6                   0
        35          6.5                 -8
        33          7                   -16
        30          8                   -32
        27          9                   -35
        24          10                  -38
        Note: Linear from 6 to 8 and 8 to 10
     */
}
