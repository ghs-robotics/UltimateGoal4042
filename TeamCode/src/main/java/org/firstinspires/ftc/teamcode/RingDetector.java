package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import java.util.*;

class RingDetector {

    /**
     * @return an array containing the xy coordinates of the ring on the screen
     * <p>
     * this function could be used to detect where the rings are on the field
     * which in turn allows Kai's ring counting code to focus on that
     * specific spot
     * <p>
     * WARNING
     * this method only works in specific lighting conditions please make sure that you
     * adjust the hsv values in the lower and upper hsv scalars
     * <p>
     * also feel free to improve upon this code!
     */
    public static int[] getRingCoordinates(Mat input) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        Scalar GREEN = new Scalar(0, 255, 0);

        Mat src = input;
        Imgproc.resize(src, src, new Size(320, 240));
        Mat dst = new Mat();

        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

        //adding a mask to the dst mat
        Scalar lowerHSV = new Scalar(103, 146, 164);
        Scalar upperHSV = new Scalar(112, 242, 255);
        Core.inRange(dst, lowerHSV, upperHSV, dst);

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

            //dont draw a square around a spot that's too small
            //to avoid false detections
            if (rect.area() > 7_000) {
                Imgproc.rectangle(src, rect, GREEN, 5);
            }
        }

        Rect largest = new Rect();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));

            if (largest.area() < rect.area()) largest = rect;
        }

        //draws largest rect
        Imgproc.rectangle(src, largest, new Scalar(0, 0, 255), 5);

        System.out.printf("x %d y %d", largest.x, largest.y);
        return new int[]{largest.x,largest.y, largest.width, largest.height};
    }
}
