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
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(320, 200), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(5, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(130, 0), new Point(320, 240), GREEN_BGR, -1);
    }

    public int findConfig() {
        updateData();
        if (5 <= h && h <= 20) { // typically about 10
            return 1;
        } else if (21 <= h && h <= 34) { // typically about 21
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
        return (4 < w && 4 < h && h < 40 && 15 < x && x < 72 && 200 < y && y < 240);
    }

    @Override
    public void resetPIDs() {}
}
