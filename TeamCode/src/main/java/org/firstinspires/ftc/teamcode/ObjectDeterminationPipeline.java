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
        //update ring coordinates
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

        return input;
    }

    //Detects the position of the target object on the screen and returns an array with those values
    public static int[] getObjectCoordinates(Mat input) {
        Scalar GREEN = new Scalar(0, 255, 0);

        Mat dst = new Mat();
        Mat src = input;
        Imgproc.resize(src, src, new Size(320, 240));

        //Cover up background noise
        Imgproc.rectangle(src, new Point(0, 0), new Point(320, (int) (Robot.cover * 240)), GREEN, -1);

        Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);
        Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

        //adding a mask to the dst mat
        Core.inRange(dst, Robot.lower, Robot.upper, dst);

        //dilate the ring to make it easier to detect
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