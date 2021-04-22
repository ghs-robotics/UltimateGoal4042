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
        cover = 0.25;
        targetY = 115;
    }

    @Override
    public void coverBackground() {
        super.coverBackground();
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(75, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(245, 0), new Point(320, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(0, (int) (0.7 * SCREEN_HEIGHT)), new Point(320, 240), GREEN_BGR, -1);
    }

    // No side to side PID
    @Override
    public double getBreadthPIDValue() {
        return 0;
    }

    @Override
    public double getDepthPIDValue() {
        if (identified) {
            return -depthPID.calcVal(getErrorY());
        } else {
            return 0;
        }
    }

    // Testing to make sure the detected object is the wall close up
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return w == 169 && 60 < y && y + h == 168;
    }
    /*
        Some useful values
        y       field pos
        66      at tower goal (furthest)
        80      pre perfect
        92      perfect launch pos
        124     2 ft from back wall
        160     1 ft from back wall (robot can barely see floor from here)
     */

    @Override
    public void resetPIDs() {
        depthPID.resetValues();
    }
}
