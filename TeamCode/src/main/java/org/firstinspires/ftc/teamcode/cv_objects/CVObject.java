package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.data.HSVConstants;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public abstract class CVObject implements HSVConstants {

    public String name = "unspecified";
    public boolean identified = false;

    public CVDetectionPipeline pipeline;

    // Contains frame that will be used for object detection
    // constantly updated (in updateData)
    protected Mat currentHSVMat;
    protected Mat hierarchy;

    public Scalar lowerHSV;
    public Scalar upperHSV;

    // All measured in pixels (on the screen)
    public int x = 0;
    public int y = 0;
    public int w = 0; // width
    public int h = 0; // height

    // Values to be targeted
    int targetX = 0;
    int targetY = 0;
    int targetW = 0;

    // Amount of screen covered by a white rectangle during image processing
    public double cover = 0;

    public PIDController xPID;
    public PIDController wPID;

    public CVObject(String name, CVDetectionPipeline pipeline) {
        this.name = name;
        this.pipeline = pipeline;
        this.currentHSVMat = new Mat();
        this.hierarchy = new Mat();
    }

    public CVObject(String name, CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        this(name, pipeline);
        this.xPID = xPID;
        this.wPID = wPID;
    }

    // Creates solid rectangles to cover up background noise
    protected void coverBackground() {
        Imgproc.rectangle(
                currentHSVMat,
                new Point(0, 0),
                new Point(SCREEN_WIDTH, (int) (cover * SCREEN_HEIGHT)),
                GREEN_BGR, -1
        );
    }

    public int getErrorX() {
        return targetX - x;
    }

    public int getErrorY() {
        return targetY - y;
    }

    public int getErrorW() {
        return targetW - w;
    }

    public double getXPIDValue() {
        if (identified) {
            return -xPID.calcVal(getErrorX());
        } else {
            return 0;
        }
    }

    public double getWPIDValue() {
        if (identified) {
            return wPID.calcVal(getErrorW());
        } else {
            return 0;
        }
    }

    public boolean isIdentified() {
        return identified;
    }

    abstract boolean isReasonable(int x, int y, int w, int h);

    public void resetPIDs() {
        xPID.resetValues();
        wPID.resetValues();
    }

    public void setTargetX(int targetX) {
        this.targetX = targetX;
    }

    public void setTargetXW(int[] vals) {
        this.targetX = vals[0];
        this.targetW = vals[1];
        resetPIDs();
    }

    public void setTargetY(int targetY) {
        this.targetY = targetY;
    }

    public void setTargetW(int targetW) {
        this.targetW = targetW;
    }

    @Override
    public String toString() {
        return name + " (x = " + x + ", y = " + y + ", w = " + w + ", h = " + h + ")";
    }

    // Updates coordinates and identified boolean
    public void updateData() {
        Mat input = pipeline.currentMat; // Edits to this Mat will display on the phone screen

        // Converts color from BGR (default format for OpenCV) to HSV (easier format to process with)
        Imgproc.cvtColor(input, currentHSVMat, Imgproc.COLOR_BGR2HSV);

        // Adds rectangles
        coverBackground();

        // Filters colors within certain color range
        Core.inRange(currentHSVMat, this.lowerHSV, this.upperHSV, currentHSVMat);

        // Finds the contours of the object and stores them in an ArrayList
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(currentHSVMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the src image TODO : DOES THIS EVEN IMPACT ANYTHING?
        Imgproc.drawContours(input, contours, -1, GREEN_BGR, 2, Imgproc.LINE_8, hierarchy, 2, new Point());

        // Creates a rectangle called rect with default value of 0 for x, y, width, and height
        Rect largestRect = new Rect();

        // Iterates through all of the contours and finds the largest bounding rectangle
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (rect.area() < rect.area() && isReasonable(rect.x, rect.y, rect.width, rect.height)) {
                largestRect = rect;
            }
        }

        // Draws largest rect on src image TODO : DOES THIS EVEN IMPACT ANYTHING?
        Imgproc.rectangle(input, largestRect, GREEN_BGR, 1);

        // Updates coordinates
        this.x = largestRect.x;
        this.y = largestRect.y;
        this.w = largestRect.width;
        this.h = largestRect.height;

        // If x, y, w, h are all zero, the object is NOT being identified
        identified = (x + y + w + h > 0);
    }
}
