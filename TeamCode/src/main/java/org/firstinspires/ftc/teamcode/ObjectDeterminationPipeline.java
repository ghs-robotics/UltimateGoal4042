package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
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
        /*String file = "D:/Downloads/BenImages/TapeLeft.jpeg";
        Imgcodecs imageCodecs = new Imgcodecs();
        Mat image = Imgcodecs.imread(file);
        int[] coordinates = getSquareCoordinates(image, 0);
        System.out.println("x: " + coordinates[0] + ", y: " + coordinates[1]); */
    }

    public static int[] getSquareCoordinatesMat ( Mat input, int squareNum) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        //we need to find the hsv values for the actual squares
        //right now we just have placeholders
        Scalar LOWER_SQUARE_HSV = new Scalar(0,0,98);
        Scalar UPPER_SQUARE_HSV = new Scalar(14,135,242);

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Scalar GREEN = new Scalar(0,255,0);
        Scalar RED = new Scalar(0,0,255);
        Scalar BLUE = new Scalar(255,0,0);

        //Make the image easier to read
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 80, 80);
        Core.inRange(input, LOWER_SQUARE_HSV, UPPER_SQUARE_HSV, input);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(input, input, kernel);

        //Find object contours for the square
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //Find the largest contours in the image
        ArrayList<Rect> rects = new ArrayList<>();
        Rect pickedRect = new Rect();

        for (int i = 0; i < 3; i++) {
            Rect largest = new Rect();
            int largestIndex = 0;
            for (int j = 0; j < contours.size(); j++) {
                Rect rect = Imgproc.boundingRect(contours.get(i));
                if(rect.area() > largest.area()) {
                    largest = rect;
                    largestIndex = j;
                }
            }
            rects.add(largest);
            contours.remove(largestIndex);
        }

        int[] xVals = new int[]{rects.get(0).x, rects.get(1).x, rects.get(2).x};
        Arrays.sort(xVals);
        pickedRect = xVals[squareNum] == rects.get(0).x ? rects.get(0) : (xVals[squareNum] == rects.get(1).x) ? rects.get(1) : rects.get(2);

        int w = pickedRect.width, h = pickedRect.height, x = pickedRect.x, y = pickedRect.y;

        Imgproc.rectangle(input, rects.get(0), RED, 5);
        Imgproc.rectangle(input, rects.get(1), RED, 5);
        Imgproc.rectangle(input, rects.get(2), RED, 5);
        Imgproc.circle(input, new Point(x + w/2, y + h/2),50, BLUE,5);

        return new int[]{pickedRect.x, pickedRect.y};
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