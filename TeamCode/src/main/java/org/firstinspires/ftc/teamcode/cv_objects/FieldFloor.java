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
        cover = 0.2; // TODO : UPDATE FOR NEW PHONE ANGLE
        targetY = 140; // h ranges from 20 (super close to back wall) to 100 (front of field)
    }

    @Override
    public void coverBackground() {
        super.coverBackground();
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(100, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(220, 0), new Point(320, 240), GREEN_BGR, -1);
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
        // y ranges from 76 (robot at tower goal) to 125 (robot 2 ft from the wall)
        return w == 119 && 73 < y && y + h == 168;
    }

    @Override
    public void resetPIDs() {
        depthPID.resetValues();
    }
}
