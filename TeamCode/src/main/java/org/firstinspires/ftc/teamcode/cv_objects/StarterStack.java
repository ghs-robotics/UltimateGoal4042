package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class StarterStack extends CVObject {

    public StarterStack(CVDetectionPipeline pipeline) {
        super("stack", pipeline);
        lowerHSV = LOWER_STACK_HSV;
        upperHSV = UPPER_STACK_HSV;
    }

    @Override
    protected void coverBackground() { // doesn't use cover
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(320, 194), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(5, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(130, 0), new Point(320, 240), GREEN_BGR, -1);

    }

    public int findConfig() {
        updateData();
        if (5 <= h && h <= 23) { // typically about 14 - 19
            return 1;
        } else if (24 <= h && h <= 38) { // typically about 29 - 33
            return 4;
        } else {
            return 0;
        }
    }

    // No PIDs
    @Override
    public double getBreadthPIDValue() {
        return 0;
    }

    // No PIDs
    @Override
    public double getDepthPIDValue() {
        return 0;
    }

    // Testing to make sure the detected object is the starter stack
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return (4 < w && 4 < h && h < 40 /*&& 15 < x && x < 72 */ && 185 < y && y < 240);
    }


    @Override
    public void resetPIDs() {}
}
