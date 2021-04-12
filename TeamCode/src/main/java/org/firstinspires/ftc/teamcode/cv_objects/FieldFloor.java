package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class FieldFloor extends CVObject {

    public FieldFloor(CVDetectionPipeline pipeline, PIDController yPID) {
        super("floor", pipeline);
        this.wPID = yPID;
        lowerHSV = LOWER_FLOOR_HSV;
        upperHSV = UPPER_FLOOR_HSV;
        cover = 0;
    }

    @Override
    protected void coverBackground() {
        super.coverBackground();
    }

    // Testing to make sure the detected object is the wall close up
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return true; // TODO : UPDATE
    }
}
