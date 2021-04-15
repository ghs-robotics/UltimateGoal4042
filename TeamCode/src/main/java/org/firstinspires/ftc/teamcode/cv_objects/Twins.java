package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

// The red twin columns bordering the mid goal
public class Twins extends CVObject {

    private Mat auxiliaryMat;

    public Twins(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("tower", pipeline, xPID, wPID);
        lowerHSV = LOWER_TWIN_HSV;
        upperHSV = UPPER_TWIN_HSV;
        cover = 0.4; // TODO : TEST
    }

    // Covers everything on the screen except the given region and a slim border
    private void coverMoreBackground(int x, int y, int w, int h) {
        // To the left of black part of tower goal
        Imgproc.rectangle(
                currentHSVMat,
                new Point(0, 0),
                new Point(Math.max(0, x - 20), SCREEN_HEIGHT),
                GREEN_BGR, -1
        );
        // To the right of black part of tower goal
        Imgproc.rectangle(
                currentHSVMat,
                new Point(Math.min(SCREEN_WIDTH, x + w + 20), 0),
                new Point(SCREEN_WIDTH, SCREEN_HEIGHT),
                GREEN_BGR, -1
        );
        // Above black part of tower goal
        Imgproc.rectangle(
                currentHSVMat,
                new Point(0, 0),
                new Point(SCREEN_WIDTH, Math.max(0, y - 20)),
                GREEN_BGR, -1
        );
        // Below black part of tower goal
        Imgproc.rectangle(
                currentHSVMat,
                new Point(0, y + h),
                new Point(SCREEN_WIDTH, SCREEN_HEIGHT),
                GREEN_BGR, -1
        );
    }

    // Testing to make sure the detected object is the tower goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        double r = 1.0 * h / w;
        return (3 < w && w < 20 && 18 < h && h < 75 && r > 2.5); // TODO : TEST
    }

    // Finds the black part of the tower goal and covers everything outside of it with rectangles
    private void restrictSearchToBlackRegion() {
        // Filters colors within certain color range and saves result in auxiliaryMat
        Core.inRange(currentHSVMat, LOWER_BLACK_TOWER_HSV, UPPER_BLACK_TOWER_HSV, auxiliaryMat);

        // Finds the contours of the object and stores them in an ArrayList
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(auxiliaryMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Creates a rectangle called rect with default value of 0 for x, y, width, and height
        Rect largestRect = new Rect();

        // Iterates through all of the contours and finds the largest bounding rectangle
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largestRect.area() < rect.area()) {
                largestRect = rect;
            }
        }

        coverMoreBackground(largestRect.x, largestRect.y, largestRect.width, largestRect.height);
    }

    // Takes the two columns and synthesizes their data into "one" identified object
    private void synthesize(Rect left, Rect right) {
        // Check if these are actually the two columns
        int deltaX = Math.abs(left.x - right.x);
        int errorY = Math.abs(left.y - right.y);
        int errorH = Math.abs(left.height - right.height);
        if (26 < deltaX && deltaX < 115 && Math.max(errorY, errorH) < 5) { // TODO : TEST
            this.x = left.x;
            this.y = left.y;
            this.w = deltaX + right.width;
            this.h = left.height;
            identified = true;
        } else {
            this.x = 0;
            this.y = 0;
            this.w = 0;
            this.h = 0;
            identified = false;
        }
    }

    // Updates coordinates and identified boolean
    // ONLY CALL THIS FROM WITHIN THE PIPELINE
    @Override
    public void updateData() {
        Mat input = pipeline.currentMat; // Edits to this Mat will display on the phone screen

        // Converts color from BGR (default format for OpenCV) to HSV (easier format to process with)
        Imgproc.cvtColor(input, currentHSVMat, Imgproc.COLOR_BGR2HSV);

        // Adds rectangles
        coverBackground();

        // Optional
//        restrictSearchToBlackRegion();

        // Filters colors within certain color range
        Core.inRange(currentHSVMat, this.lowerHSV, this.upperHSV, currentHSVMat);

        // Finds the contours of the object and stores them in an ArrayList
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(currentHSVMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the src image
        Imgproc.drawContours(input, contours, -1, GREEN_BGR, 1, Imgproc.LINE_8, hierarchy, 2, new Point());

        // Creates a rectangle called rect with default value of 0 for x, y, width, and height
        Rect rect1 = new Rect();
        Rect rect2 = new Rect();

        // Iterates through all of the contours and finds the largest bounding rectangle
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (rect1.area() < rect.area() && isReasonable(rect.x, rect.y, rect.width, rect.height)) {
                rect1 = rect;
            }
        }

        contours.remove(rect1);

        // Iterates through all of the contours apart from rect1 and finds the largest bounding rectangle
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (rect2.area() < rect.area() && isReasonable(rect.x, rect.y, rect.width, rect.height)) {
                rect2 = rect;
            }
        }

        // Draws largest rectangles on src image
        Imgproc.rectangle(input, rect1, GREEN_BGR, 2);
        Imgproc.rectangle(input, rect2, GREEN_BGR, 2);

        if (rect1.x < rect2.x) {
            synthesize(rect1, rect2);
        } else {
            synthesize(rect2, rect1);
        }
    }
}
