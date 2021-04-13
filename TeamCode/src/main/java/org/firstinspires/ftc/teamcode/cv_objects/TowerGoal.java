package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class TowerGoal extends CVObject {

    public TowerGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("tower", pipeline, xPID, wPID);
        lowerHSV = LOWER_TOWER_HSV;
        upperHSV = UPPER_TOWER_HSV;
        cover = 0;
    }

    // Testing to make sure the detected object is the tower goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        // width 34 is back of the field, closest is 150
        return (25 < w && w < 150);
    }
}
