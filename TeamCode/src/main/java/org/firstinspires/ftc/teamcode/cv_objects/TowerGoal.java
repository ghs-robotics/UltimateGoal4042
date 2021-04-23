package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.data.MyScalar;
import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class TowerGoal extends CVObject {

    public TowerGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("tower", pipeline, xPID, wPID);
        lowerHSV = LOWER_BLUE_TOWER_HSV;
        upperHSV = UPPER_BLUE_TOWER_HSV;
        cover = 0;
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
        return (30 < w && w < 130 && 18 < h && h < 65 && r > 1.3) && hasHatShape(x, y, w, h);
    }
    /*
        Some values for reference:
        w       h
        34      22
        52      34
        77      49
        110     45 (top of goal is out of view)
     */
}
