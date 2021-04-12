package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class WobbleGoal extends CVObject {

    public WobbleGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("wobble", pipeline, xPID, wPID);
        super.coverBackground();
        lowerHSV = LOWER_WOBBLE_HSV;
        upperHSV = UPPER_WOBBLE_HSV;
        cover = 0.65;
    }

    @Override
    protected void coverBackground() {
    }

    // Testing to make sure the detected object is a wobble goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return (40 < w && w < 95);
    }
}
