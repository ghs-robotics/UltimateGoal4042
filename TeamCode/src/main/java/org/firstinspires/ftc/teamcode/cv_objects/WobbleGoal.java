package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class WobbleGoal extends CVObject {

    public WobbleGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("wobble", pipeline, xPID, wPID);
        super.coverBackground();
        lowerHSV = LOWER_WOBBLE_HSV;
        upperHSV = UPPER_WOBBLE_HSV;
    }

    @Override
    protected void coverBackground() {
        Imgproc.rectangle(currentHSVMat, new Point(0, 0), new Point(320, 40), GREEN_BGR, -1); // TODO : CHECK
        Imgproc.rectangle(currentHSVMat, new Point(0, 60), new Point(320, 240), GREEN_BGR, -1);
    }

    // No depth PID
    @Override
    public double getDepthPIDValue() {
        return 0;
    }

    // Testing to make sure the detected object is a wobble goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
//        return w < 30; // TODO : UPDATE
        return true;
    }

    @Override
    public void resetPIDs() {
        breadthPID.resetValues();
    }
}
