package org.firstinspires.ftc.teamcode.robot_components;

import org.firstinspires.ftc.teamcode.robot_components.Robot;
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

public class ObjectDeterminationPipeline extends OpenCvPipeline {
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final int SCREEN_HEIGHT = 240;
    public static final int SCREEN_WIDTH = 320;

    public int objectX = 0;
    public int objectY = 0;
    public int objectWidth = 0;
    public int objectHeight = 0;

    @Override
    public void init(Mat firstFrame) {
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
                -1); // Negative thickness means solid fill
        input = showHSVCrosshair(input);

        return input;
    }
    //https://stackoverflow.com/questions/17035005/using-get-and-put-to-access-pixel-values-in-opencv-for-java
    //this is the method that displays hsv values of a point on screen
    private Mat showHSVCrosshair(Mat input) {
        int targetX = input.cols()/2;
        int targetY = input.rows()/2;

        Mat dst = input.clone();
        dst.convertTo(dst, CvType.CV_64FC3);
        Imgproc.cvtColor(dst,dst,Imgproc.COLOR_RGB2HSV);
        int size = (int) (input.total() * input.channels());

        double[] data = new double[size];

        dst.get(targetX,targetY,data);


        input.get(targetX,targetY);

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

        //convert color from BGR to HSV
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