package org.firstinspires.ftc.teamcode.robot_components;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CVDetectionPipeline extends OpenCvPipeline implements HSVConstants {

    // Some constants for the screen size
    public static final int SCREEN_HEIGHT = 240;
    public static final int SCREEN_WIDTH = 320;

    // Detects rings by default to start out
    public Scalar lowerHSV = LOWER_RING_HSV;
    public Scalar upperHSV = UPPER_RING_HSV;

    public String targetObject = "ring";

    // Coordinates of detected object (these values are constantly updated)
    public int objectX = 0;
    public int objectY = 0;
    public int objectWidth = 0;
    public int objectHeight = 0;

    // Amount of screen covered by a white rectangle during image processing
    public double cover = 0;

    public String crosshairValue = "";

    // Auxiliary Mat objects for temporarily storing data
    private Mat dst = new Mat();
    private Mat hierarchy = new Mat();

    // Returns array with coordinates of detected object, allowing us to access the data elsewhere
    public int[] getObjectData() {
        return new int[]{objectX, objectY, objectWidth, objectHeight};
    }

    // This method is called in the background (not by us)
    @Override
    public void init(Mat firstFrame) {
//        inputToCb(firstFrame);
//        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    // Processes image before displaying on phone screen
    // This method is called in the background (not by us) every time the camera receives a new
    // input, which happens multiple times a second while we're streaming
    @Override
    public Mat processFrame(Mat input) {
        // Update object coordinates
        int[] vals = findObjectCoordinates(input);
        objectX = vals[0];
        objectY = vals[1];
        objectWidth = vals[2];
        objectHeight = vals[3];

        // TODO : COMMENT OUT
        crosshairValue = findHSVCrosshair(input);

        return input;
    }

    // Checks if the data for the target object is reasonable
    public boolean checkIfReasonable(String target, int x, int y, int w, int h) {
        if (target.equals("ring")) {
            return passesRingTest(w, h);
        }
        else if (target.equals("stack")) {
            return passesStackTest(x, w, h);
        }
        else if (target.equals("tower")) {
            return passesTowerTest(w);
        }
        else if (target.equals("wobble")) {
            return passesWobbleTest(w, h);
        }
        return false; // Return false if none of the above targets apply
    }

    // Updates the lower and upper HSV values as well as the cover
    public void setTargetTo(String target) throws IllegalArgumentException {
        if (target.equals("ring")) {
            lowerHSV = LOWER_RING_HSV;
            upperHSV = UPPER_RING_HSV;
            cover = 0.65; // TODO : CALIBRATE
        }
        else if (target.equals("stack")) {
            lowerHSV = LOWER_STACK_HSV;
            upperHSV = UPPER_STACK_HSV;
            cover = 0.65;
        }
        else if (target.equals("tower")) {
            lowerHSV = LOWER_TOWER_HSV;
            upperHSV = UPPER_TOWER_HSV;
            cover = 0;
        }
        else if (target.equals("wobble")) {
            lowerHSV = LOWER_WOBBLE_HSV;
            upperHSV = UPPER_WOBBLE_HSV;
            cover = 0.65;
        }
        else if (target.equals("wall")) {
            lowerHSV = LOWER_WALL_HSV;
            upperHSV = UPPER_WALL_HSV;
            cover = 0;
        }
        else {
            throw new IllegalArgumentException("Target must be ring, tower, or wobble!");
        }
        targetObject = target;
    }

    // Displays HSV values of a point on the screen
    // More info: https://stackoverflow.com/questions/17035005/using-get-and-put-to-access-pixel-values-in-opencv-for-java
    private String findHSVCrosshair(Mat input) {
        int col = input.cols()/2;
        int row = input.rows()/2;

        dst = input.clone();
//        dst.convertTo(dst, CvType.CV_64FC3);
        Imgproc.cvtColor(dst,dst,Imgproc.COLOR_BGR2HSV);

        dst = dst.row(row);
        dst = dst.col(col);
        String value = dst.dump();
        dst.release(); // Avoid leaking memory

        return value;
    }

    // Detects the position of the target object on the screen and returns an array with those values
    public int[] findObjectCoordinates(Mat src) {

        // Imgproc is a class that comes with the library and has a bunch of useful methods
        // Resizes image to make processing more uniform
        Imgproc.resize(src, src, new Size(320, 240));

        // Covers up background noise
        // Creates rectangle
        Imgproc.rectangle(src, new Point(0, 0), new Point(320, (int) (cover * 240)), GREEN_BGR, -1);

        // Covers up most of screen if analyzing starter stack
        if (targetObject.equals("stack")) {
            Imgproc.rectangle(src, new Point(0, 0), new Point(140, 240), GREEN_BGR, -1);
            Imgproc.rectangle(src, new Point(270, 0), new Point(320, 240), GREEN_BGR, -1);
        }

        // Converts color from BGR (default format for OpenCV) to HSV (easier format to process with)
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);

        // Filters colors within certain color range
        Core.inRange(dst, lowerHSV, upperHSV, dst);

        // Finds the contours of the object and stores them in an ArrayList
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the src image
        Imgproc.drawContours(src, contours, -1, GREEN_BGR, 2, Imgproc.LINE_8, hierarchy, 2, new Point());

        // Creates a rectangle called rect with default value of 0 for x, y, width, and height
        Rect largestRect = new Rect();

        // Iterates through all of the contours and finds the largest bounding rectangle
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largestRect.area() < rect.area() && checkIfReasonable(targetObject, rect.x, rect.y, rect.width, rect.height)) {
                largestRect = rect;
            }
        }

        // Draws largest rect on src image
        Imgproc.rectangle(src, largestRect, GREEN_BGR, 1); // TODO : comment out?

        // Returns the data for the largest rectangle that is still reasonable
        return new int[]{largestRect.x, largestRect.y, largestRect.width, largestRect.height};
    }


    // Testing to make sure the detected object is a ring
    private boolean passesRingTest(double w, double h) {
        double r = 1.0 * w / h; // ratio
        return (h > 8 && h < 23 && w > 32 && w < 90 && r > 1.5 && r < 5); // 8,23,22,90,1.5,7
    }

    // Testing to make sure the detected object is a ring stack
    private boolean passesStackTest(double x, double w, double h) {
        return (4 < w && 4 < h && h < 40 && x > 0);
    }

    // Testing to make sure the detected object is the tower goal
    private boolean passesTowerTest(double w) {
        // width 34 is back of the field, closest is 150
        return (25 < w && w < 150);
    }

    // Testing to make sure the detected object is the wall close up
    private boolean passesWallTest(double w, double h) {
        return (w > 1 && w < 12 && h > 8 && h < 23); // TODO : PLACEHOLDERS
    }

    // Testing to make sure the detected object is a wobble goal
    private boolean passesWobbleTest(double w, double h) {
        return (40 < w && w < 95);
    }

    // A comprehensive backup
    public int[] findObjectCoordinates2(Mat src) {
        Scalar GREEN = new Scalar(0, 255, 0);

        Imgproc.resize(src, src, new Size(320, 240));

        //Cover up background noise
        //creates rectangle
        Imgproc.rectangle(src, new Point(0, 0), new Point(320, (int) (cover * 240)), GREEN, -1);

        //convert color from BGR to HSV
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);
        //create Gaussian blur on image
        Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

        //adding a mask to the dst mat
        //filters colors within certain color range
        Core.inRange(dst, lowerHSV, upperHSV, dst);

        //dilate the ring to make it easier to detect
        //kernel determines how much you are changing the pixel
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(dst, dst, kernel);

        //get the contours of the ring
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw a contour on the src image
        Imgproc.drawContours(src, contours, -1, GREEN, 2, Imgproc.LINE_8, hierarchy, 2, new Point());

        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));

            //don't draw a square around a spot that's too small
            //to avoid false detections
            //if (rect.area() > 7_000) { Imgproc.rectangle(src, rect, GREEN, 5); }
        }

        Rect largest = new Rect();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largest.area() < rect.area()) {
                largest = rect;
            }
        }

        //draws largest rect
        Imgproc.rectangle(src, largest, GREEN, 5);

        return new int[]{largest.x, largest.y, largest.width, largest.height};
    }










//    // To classify starter stack
//
//    /*
//     * An enum to define the ring position
//     */
//    public enum RingPosition
//    {
//        FOUR,
//        ONE,
//        ZERO
//    }
//
//    /*
//     * Some color constants
//     */
//    static final Scalar BLUE = new Scalar(0, 0, 255);
//    static final Scalar GREEN = new Scalar(0, 255, 0);
//
//    /*
//     * The core values which define the location and size of the sample regions
//     */
//    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200,160);
//
//    static final int REGION_WIDTH = 35;
//    static final int REGION_HEIGHT = 25;
//
//    final int FOUR_RING_THRESHOLD = 150;
//    final int ONE_RING_THRESHOLD = 135;
//
//    Point region1_pointA = new Point(
//            REGION1_TOPLEFT_ANCHOR_POINT.x,
//            REGION1_TOPLEFT_ANCHOR_POINT.y);
//    Point region1_pointB = new Point(
//            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//    /*
//     * Working variables
//     */
//
//    Mat region1_Cb;
//    Mat YCrCb = new Mat();
//    Mat Cb = new Mat();
//    int avg1;
//
//    // Volatile since accessed by OpMode thread w/o synchronization
//    private volatile ClassifyStarterStack.RingDeterminationPipeline.RingPosition position = ClassifyStarterStack.RingDeterminationPipeline.RingPosition.FOUR;
//
//    /*
//     * This function takes the RGB frame, converts to YCrCb,
//     * and extracts the Cb channel to the 'Cb' variable
//     */
//    void inputToCb(Mat input)
//    {
//        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//        Core.extractChannel(YCrCb, Cb, 1);
//    }
//
//    private Mat processStarterStack(Mat input)
//    {
//        inputToCb(input);
//
//        avg1 = (int) Core.mean(region1_Cb).val[0];
//
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region1_pointA, // First point which defines the rectangle
//                region1_pointB, // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
//
//        position = ClassifyStarterStack.RingDeterminationPipeline.RingPosition.FOUR; // Record our analysis
//        if(avg1 > FOUR_RING_THRESHOLD) {
//            position = ClassifyStarterStack.RingDeterminationPipeline.RingPosition.FOUR;
//        } else if (avg1 > ONE_RING_THRESHOLD) {
//            position = ClassifyStarterStack.RingDeterminationPipeline.RingPosition.ONE;
//        } else {
//            position = ClassifyStarterStack.RingDeterminationPipeline.RingPosition.ZERO;
//        }
//
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region1_pointA, // First point which defines the rectangle
//                region1_pointB, // Second point which defines the rectangle
//                GREEN, // The color the rectangle is drawn in
//                1); // Negative thickness means solid fill
//
//        return input;
//    }
//
//    public int getAnalysis() {
//        return avg1;
//    }

}