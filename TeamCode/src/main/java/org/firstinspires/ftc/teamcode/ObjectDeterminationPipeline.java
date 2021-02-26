package org.firstinspires.ftc.teamcode;

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
import java.util.Arrays;

public class ObjectDeterminationPipeline extends OpenCvPipeline {
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar LOWER_SQUARE_HSV = new Scalar(0, 0, 0);
    public static final Scalar UPPER_SQUARE_HSV = new Scalar(255, 255, 255);

    public static final int SCREEN_HEIGHT = 240;
    public static final int SCREEN_WIDTH = 320;

    public int objectX = 0;
    public int objectY = 0;
    public int objectWidth = 0;
    public int objectHeight = 0;

    @Override
    public void init(Mat firstFrame) {
    }

    //This function is a WIP, still needs testing
    public int[] getSquareCoordinates(Mat input, int squareNum) {
        Mat src = input;
        int squareX = 0;
        int squareY = 0;
        //Make the image easier to read
        Imgproc.resize(src, src, new Size(320, 240));
        Imgproc.GaussianBlur(src, src, new Size(5, 5), 80, 80);
        Core.inRange(src, LOWER_SQUARE_HSV, UPPER_SQUARE_HSV, src);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(src, src, kernel);

        //Find object contours for the square
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(src, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //Find the largest contours in the image
        Rect largest = new Rect();
        Rect largest2 = new Rect();
        Rect largest3 = new Rect();

        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largest.area() < rect.area()) {
                largest = rect;
            } else if (largest2.area() < rect.area()) {
                largest2 = rect;
            } else if (largest3.area() < rect.area()) {
                largest3 = rect;
            }
        }
        Imgproc.rectangle(src, largest, GREEN, 5);
        Imgproc.rectangle(src, largest2, GREEN, 5);
        Imgproc.rectangle(src, largest3, GREEN, 5);
        int x1 = largest.x; int x2 = largest2.x; int x3 = largest3.x;
        int[] xVals = new int[]{x1,x2,x3};
        Arrays.sort(xVals);
        if (squareNum == 0) {
            squareX = (xVals[0] == largest.x) ? largest.x : (xVals[0] == largest2.x) ? largest2.x : largest3.x;
            squareY = (xVals[0] == largest.x) ? largest.y : (xVals[0] == largest2.x) ? largest2.y : largest3.y;
        } else if (squareNum == 1) {
            squareX = (xVals[1] == largest.x) ? largest.x : (xVals[1] == largest2.x) ? largest2.x : largest3.x;
            squareY = (xVals[1] == largest.x) ? largest.y : (xVals[1] == largest2.x) ? largest2.y : largest3.y;
        } else {
            squareX = (xVals[2] == largest.x) ? largest.x : (xVals[2] == largest2.x) ? largest2.x : largest3.x;
            squareX = (xVals[2] == largest.x) ? largest.y : (xVals[2] == largest2.x) ? largest2.y : largest3.y;
        }
        Imgproc.circle(src, new Point(squareX, squareY),50, GREEN,5);

        return new int[]{squareX, squareY};
    }

    @Override
    public Mat processFrame(Mat input) {
        // Update ring coordinates
        int[] coords = getObjectCoordinates(input);
        objectX = coords[0];
        objectY = coords[1];
        objectWidth = coords[2];
        objectHeight = coords[3];

        Imgproc.rectangle(
                input, // Buffer to draw on
                new Point(objectX, objectY), // First point which defines the rectangle
                new Point(objectX + objectWidth, objectY + objectHeight), // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Negative thickness (-1) means solid fill

        return input;
    }

    public static int[] getObjectCoordinates(Mat src) {
        Mat dst = new Mat();
        Imgproc.resize(src, src, new Size(320, 240));

        // Convert color from RGB to HSV
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);

        // adding a mask to the dst mat
        // filters colors within certain color range
        Core.inRange(dst, Robot.lower, Robot.upper, dst);

        // Get the contours of the ring
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the src image
        Imgproc.drawContours(src, contours, -1, GREEN, 2, Imgproc.LINE_8, hierarchy, 2, new Point());

        Rect largest = new Rect();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largest.area() < rect.area()) {
                largest = rect;
            }
        }

        return new int[]{largest.x, largest.y, largest.width, largest.height};
    }


    // Detects the position of the target object on the screen and returns an array with those values
    public static int[] getObjectCoordinates2(Mat src) {
        Scalar GREEN = new Scalar(0, 255, 0);

        Mat dst = new Mat();
        Imgproc.resize(src, src, new Size(320, 240));

        //Cover up background noise
        //creates rectangle
        Imgproc.rectangle(src, new Point(0, 0), new Point(320, (int) (Robot.cover * 240)), GREEN, -1);

        //convert color from RGB to HSV
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);
        //create Gaussian blur on image
        Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

        //adding a mask to the dst mat
        //filters colors within certain color range
        Core.inRange(dst, Robot.lower, Robot.upper, dst);

        //dilate the ring to make it easier to detect
        //kernel determines how much you are changing the pixel
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(dst, dst, kernel);

        //get the contours of the ring
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
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