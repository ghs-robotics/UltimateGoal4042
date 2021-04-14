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
        cover = 0.35;
        targetH = 80; // Good position to pick up 2nd wobble goal in auto
    }

    @Override
    protected void coverBackground() {
        super.coverBackground();
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(120, 240), GREEN_BGR, -1);
        Imgproc.rectangle(currentHSVMat, new Point(200, 0), new Point(320, 240), GREEN_BGR, -1);
    }

    // No side to side PID
    @Override
    public double getBreadthPIDValue() {
        return 0;
    }

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
        // h ranges from 23 (across field) to 110 (very close to wall)
        return w == 79 && 16 < h && h < 130 && y > 80;
    }

    @Override
    public void resetPIDs() {
        depthPID.resetValues();
    }
}
