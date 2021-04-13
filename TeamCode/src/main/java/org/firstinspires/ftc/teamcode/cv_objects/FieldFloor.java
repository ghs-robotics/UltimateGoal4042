package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class FieldFloor extends CVObject {

    public FieldFloor(CVDetectionPipeline pipeline, PIDController yPID) {
        super("floor", pipeline);
        this.depthPID = yPID;
        lowerHSV = LOWER_FLOOR_HSV;
        upperHSV = UPPER_FLOOR_HSV;
        cover = 0.45; // TODO : UPDATE FOR NEW PHONE ANGLE
    }

    // No side to side PID
    @Override
    public double getBreadthPIDValue() {
        return 0;
    }

    @Override
    public double getDepthPIDValue() {
        if (identified) {
            return depthPID.calcVal(getErrorY());
        } else {
            return 0;
        }
    }

    // Testing to make sure the detected object is the wall close up
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return true; // TODO : UPDATE
    }

    @Override
    public void resetPIDs() {
        depthPID.resetValues();
    }
}
