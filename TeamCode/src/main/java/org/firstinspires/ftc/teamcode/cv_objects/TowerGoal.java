package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class TowerGoal extends CVObject {

    public TowerGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("tower", pipeline, xPID, wPID);
        lowerHSV = LOWER_BLUE_TOWER_HSV;
        upperHSV = UPPER_BLUE_TOWER_HSV;
        cover = 0;
    }

    // Testing to make sure the detected object is the tower goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        double r = 1.0 * w / h; // ratio is usually about 1.5
        // width 34 is back of the field, closest is 150
        return (30 < w && w < 115 && 18 < h && h < 55 && r > 1.3);
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
