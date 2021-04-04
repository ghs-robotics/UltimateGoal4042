package org.firstinspires.ftc.teamcode.robot_components;

import org.opencv.core.Core;

import org.opencv.core.CvType;
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

    public static final int SCREEN_HEIGHT = 240;
    public static final int SCREEN_WIDTH = 320;

    // Detects rings by default
    public Scalar lowerHSV = LOWER_RING_HSV;
    public Scalar upperHSV = UPPER_RING_HSV;

    public String targetObject = "ring";

    // Coordinates of detected object
    public int objectX = 0;
    public int objectY = 0;
    public int objectWidth = 0;
    public int objectHeight = 0;

    // Amount of screen covered by a white rectangle during image processing
    public double cover = 0;

    // Auxiliary Mat objects
    private Mat dst = new Mat();
    private Mat hierarchy = new Mat();

    // Returns array with coordinates of detected object
    public int[] getObjectData() {
        return new int[]{objectX, objectY, objectWidth, objectHeight};
    }

    @Override
    public void init(Mat firstFrame) {}

    // Processes image before displaying on phone screen
    @Override
    public Mat processFrame(Mat input) {
        // Update object coordinates
        int[] vals = findObjectCoordinates(input);
        objectX = vals[0];
        objectY = vals[1];
        objectWidth = vals[2];
        objectHeight = vals[3];

//        input = showHSVCrosshair(input); //TODO : uncomment?

        return input;
    }

    public void setTargetTo(String target) throws IllegalArgumentException {
        if (target.equals("ring")) {
            lowerHSV = LOWER_RING_HSV;
            upperHSV = UPPER_RING_HSV;
            cover = 0.65;
        }
        else if (target.equals("stack")) {
            lowerHSV = LOWER_STACK_HSV;
            upperHSV = UPPER_STACK_HSV;
            cover = 0.70;
        }
        else if (target.equals("tower")) {
            lowerHSV = LOWER_TOWER_HSV;
            upperHSV = UPPER_TOWER_HSV;
            cover = 0;
        }
        else if (target.equals("wobble")) {
            lowerHSV = LOWER_WOBBLE_HSV;
            upperHSV = UPPER_WOBBLE_HSV;
            cover = 0.63;
        }
        else {
            throw new IllegalArgumentException("Target must be ring, tower, or wobble!");
        }
        targetObject = target;
    }

    //https://stackoverflow.com/questions/17035005/using-get-and-put-to-access-pixel-values-in-opencv-for-java
    //this is the method that displays hsv values of a point on screen
    private Mat showHSVCrosshair(Mat input) {
        int targetX = input.cols()/2;
        int targetY = input.rows()/2;

        dst = input.clone();
        dst.convertTo(dst, CvType.CV_64FC3);
        Imgproc.cvtColor(dst,dst,Imgproc.COLOR_RGB2HSV);
        int size = (int) (input.total() * input.channels());

        double[] data = new double[size];

        dst.get(targetX,targetY,data);
        input.get(targetX,targetY);

        return input;
    }

    // Detects the position of the target object on the screen and returns an array with those values
    public int[] findObjectCoordinates(Mat src) {
        Imgproc.resize(src, src, new Size(320, 240));

        //Cover up background noise
        //creates rectangle
        Imgproc.rectangle(src, new Point(0, 0), new Point(320, (int) (cover * 240)), GREEN_BGR, -1);

        // Convert color from RGB to HSV
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);

        // adding a mask to the dst mat
        // filters colors within certain color range
        Core.inRange(dst, lowerHSV, upperHSV, dst);

        // Get the contours of the ring
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the src image
        Imgproc.drawContours(src, contours, -1, GREEN_BGR, 2, Imgproc.LINE_8, hierarchy, 2, new Point());

        Rect largest = new Rect();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largest.area() < rect.area()) {

                // Check which object should be found and make sure it has a reasonable size
                if (targetObject.equals("ring")) {
                    if (passesRingTest(rect.width, rect.height)) {
                        largest = rect;
                    }
                }
                else if (targetObject.equals("stack")) {
                    if (passesStackTest(rect.width, rect.height)) {
                        largest = rect;
                    }
                }
                else if (targetObject.equals("tower")) {
                    if (passesTowerTest(rect.width)) {
                        largest = rect;
                    }
                }
                else if (targetObject.equals("wobble")) {
                    if (passesWobbleTest(rect.width, rect.height)) {
                        largest = rect;
                    }
                }
            }
        }

        // Draw largest rect
        Imgproc.rectangle(src, largest, GREEN_BGR, 1); // TODO : comment out?

        return new int[]{largest.x, largest.y, largest.width, largest.height};
    }

    // Testing to make sure the detected object is a ring
    private boolean passesRingTest(double w, double h) {
        double r = 1.0 * w / h;
        return (h > 8 && h < 23 && w > 32 && w < 90 && r > 1.5 && r < 5); // 8,23,22,90,1.5,7
    }

    // Testing to make sure the detected object is a ring stack
    private boolean passesStackTest(double w, double h) {
        return (4 < w && 4 < h && h < 35);
    }

    // Testing to make sure the detected object is the tower goal
    private boolean passesTowerTest(double w) {
        // width 34 is back of the field, closest is 150
        return true;
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
}