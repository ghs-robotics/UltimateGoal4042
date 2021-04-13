package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class StarterStack extends CVObject {

    public StarterStack(CVDetectionPipeline pipeline) {
        super("stack", pipeline);
        lowerHSV = LOWER_STACK_HSV;
        upperHSV = UPPER_STACK_HSV;
        cover = 0.65;
    }

    @Override
    protected void coverBackground() {
        super.coverBackground();
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(140, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(270, 0), new Point(320, 240), GREEN_BGR, -1);
    }

    public int findConfig() {
        activate();
        updateData();
        if (5 <= h && h <= 18) { // typically about 10
            return 1;
        } else if (19 <= h && h <= 30) { // typically about 21
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
        return (4 < w && 4 < h && h < 40 && x > 0);
    }

    @Override
    public void resetPIDs() {}
}
