package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class FieldFloor extends CVObject {

    public FieldFloor(CVDetectionPipeline pipeline, PIDController yPID) {
        super("floor", pipeline);
        this.depthPID = yPID;
        lowerHSV = LOWER_FLOOR_HSV;
        upperHSV = UPPER_FLOOR_HSV;
        cover = 0.45; // TODO : UPDATE FOR NEW PHONE ANGLE
        targetH = 60; // h ranges from 20 (super close to back wall) to 100 (front of field)
    }

    @Override
    public void coverBackground() {
        super.coverBackground();
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(100, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(220, 0), new Point(320, 240), GREEN_BGR, -1);
    }

    // No side to side PID
    @Override
    public double getBreadthPIDValue() {
        return 0;
    }

    @Override
    public double getDepthPIDValue() {
        if (identified) {
            return depthPID.calcVal(getErrorH());
        } else {
            return 0;
        }
    }

    // Testing to make sure the detected object is the wall close up
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return w == 119 && y > 125 && y + h == 240; // TODO : UPDATE
    }

    @Override
    public void resetPIDs() {
        depthPID.resetValues();
    }
}
