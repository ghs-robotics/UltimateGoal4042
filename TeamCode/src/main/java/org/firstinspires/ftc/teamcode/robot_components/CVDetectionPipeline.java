package org.firstinspires.ftc.teamcode.robot_components;

import org.firstinspires.ftc.teamcode.cv_objects.CVObject;
import org.firstinspires.ftc.teamcode.data.HSVConstants;
import org.firstinspires.ftc.teamcode.data.MyScalar;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Scanner;

public class CVDetectionPipeline extends OpenCvPipeline implements HSVConstants {

    // Stores the most recent frame
    public Mat currentMat = new Mat();

    // Auxiliary Mat objects for temporarily storing data
    private static Mat dst1 = new Mat();
    private static Mat dst2 = new Mat();
    private static Mat dst3 = new Mat();

    public ArrayList<CVObject> activeObjects;

    // For sampling HSV values of individual pixels
    public String crosshairHSV = "";

    // This method is called in the background (not by us)
    @Override
    public void init(Mat firstFrame) {}

    // Processes image before displaying on phone screen
    // This method is called in the background (not by us) every time the camera receives a new
    // input, which happens multiple times a second while we're streaming
    @Override
    public Mat processFrame(Mat input) {

        // Resizes image to make processing more uniform
        Imgproc.resize(input, input, new Size(320, 240));

        currentMat = input;
        for (CVObject obj : activeObjects) {
            obj.updateData();
        }

        // Updates crosshairValue
        crosshairHSV = findCenterHSVCrosshair(input);

        return currentMat;
    }

    // Finds HSV values of the point at the center of the screen
    public static String findHSVCrosshairString(Mat input, int column, int row) {
        // Converts to HSV
        Imgproc.cvtColor(input, dst1, Imgproc.COLOR_BGR2HSV);

        // Extracting HSV value from center point of the input image
        dst2 = dst1.row(row);
        dst3 = dst2.col(column);
        String value = dst3.dump();

        return value;
    }

    // Finds HSV values of the point at the center of the screen
    public static MyScalar findHSVCrosshair(Mat input, int row, int column) {
        // Converts to HSV
        Imgproc.cvtColor(input, dst1, Imgproc.COLOR_BGR2HSV);
        return new MyScalar(input.get(row, column)); // get returns a double[] array
    }

    // Finds HSV values of the point at the center of the screen
    public static String findCenterHSVCrosshair(Mat input) {
        int middleColumn = input.cols()/2;
        int middleRow = input.rows()/2;
        return findHSVCrosshair(input, middleColumn, middleRow).toString();
    }

    public static MyScalar cvtString2Scalar(String s) {
        Scanner text = new Scanner(s);
        int count = 0;
        int[] val = new int[3];
        while (text.hasNext()) {
            if (text.hasNextInt()) {
                val[count] = text.nextInt();
                count++;
            } else {
                text.next();
            }
        }
        return new MyScalar(val[0], val[1], val[2]);
    }

    public static MyScalar getScalar(Mat input, int column, int row) {
        return cvtString2Scalar(findHSVCrosshairString(input, column, row));
    }

    /*

    // A comprehensive backup
    public int[] findObjectCoordinates2(Mat src) {
        Scalar GREEN = new Scalar(0, 255, 0);

        Imgproc.resize(src, src, new Size(320, 240));

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

     */
}