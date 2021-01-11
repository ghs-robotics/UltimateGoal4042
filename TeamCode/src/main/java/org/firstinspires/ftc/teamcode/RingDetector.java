package org.firstinspires.ftc.teamcode;

class RingDetector {

    /**
     *
     * @return an array containing the xy coordinates of the ring on the screen
     *
     * this function could be used to detect where the rings are on the field
     * which in turn allows Kai's ring counting code to focus on that
     * specific spot
     *
     * WARNING
     * this method only works in specific lighting conditions please make sure that you
     * adjust the hsv values in the lower and upper hsv scalars
     *
     * also feel free to improve upon this code!
     */
    public int[] getCoordsOfRing() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        Scalar GREEN = new Scalar(0, 255, 0);

        //src should be an image from the phone cam
        Mat src = Imgcodecs.imread("C:\\Users\\Gamer\\Downloads\\screenshot.png");
        Mat dst = new Mat();

        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(dst, dst, new Size(5,5), 80, 80);

        //these hsv values in the scalars are only suitable for a certain lighting condition
        //adjustments need to be made
        Scalar lowerHSV = new Scalar(103,146,164);
        Scalar upperHSV = new Scalar(130,242,255);
        Core.inRange(dst, lowerHSV, upperHSV,dst);

        //dilate the ring to make it easier to detect
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
        Imgproc.dilate(dst,dst, kernel);

        //get the contours of the ring
        List<MatOfPoint> contours = new ArrayList<>();
        Mat heirarchy = new Mat();
        Imgproc.findContours(dst,contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //get the largest patch of yellow-orangy color
        Rect largest = new Rect();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));

            if(largest.area() < rect.area()) largest = rect;
        }
        return new int[]{largest.x, largest.y};
    }
}
