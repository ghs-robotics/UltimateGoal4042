package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;

public class Ring extends CVObject {

    public Ring(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("ring", pipeline, xPID, wPID);
        lowerHSV = LOWER_RING_HSV;
        upperHSV = UPPER_RING_HSV;
        cover = 0.65; // TODO : CALIBRATE
    }

    // Testing to make sure the detected object is a ring
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        double r = 1.0 * w / h; // ratio
        return (h > 8 && h < 23 && w > 32 && w < 90 && r > 1.5 && r < 5); // 8,23,22,90,1.5,7
    }
}
