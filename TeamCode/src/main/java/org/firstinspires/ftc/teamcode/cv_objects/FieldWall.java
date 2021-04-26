package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class FieldWall extends CVObject {

    public FieldWall(CVDetectionPipeline pipeline, PIDController hPID) {
        super("wall", pipeline);
        this.depthPID = hPID;
        lowerHSV = LOWER_WALL_HSV;
        upperHSV = UPPER_WALL_HSV;
        cover = 0.10;
        targetH = 70;
    }

    @Override
    protected void coverBackground() {
        super.coverBackground();
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(154, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(166, 0), new Point(320, 240), GREEN_BGR, -1);
    }

    // No side to side PID
    @Override
    public double getDepthPIDValue() {
        if (identified) {
            return -depthPID.calcVal(getErrorH());
        } else {
            return 0;
        }
    }

    // Testing to make sure the detected object is the wall close up
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        // h ranges from 28 (across field) to 110 (very close to wall); 92 is 2 ft from wall
        return w == 11 && 15 < h && h < 130;
    }
    /*
        Some useful values
        h       field pos
        23      at tower goal (furthest)
        35      pre perfect
        46      perfect launch pos
        91      2 ft from back wall
        125     1.3 ft from back wall (don't get any closer!!!)
     */

    @Override
    public void resetPIDs() {
        depthPID.resetValues();
    }
}
