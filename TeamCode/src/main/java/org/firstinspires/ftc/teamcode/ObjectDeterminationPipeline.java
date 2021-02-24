package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

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
        this.input = input;
        //update ring coordinates
        int[] coords = Robot.getObjectCoordinates(input);
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
}