package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class DistanceFinder extends LinearOpMode {

    private OpenCvInternalCamera phoneCam;
    private FindRings.RingDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new FindRings.RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("opmode is working","");
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline {

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile FindRings.RingDeterminationPipeline.RingPosition position = FindRings.RingDeterminationPipeline.RingPosition.FOUR;

        @Override
        public void init(Mat firstFrame) {
        }

        @Override
        public Mat processFrame(Mat input) {

            return getDistance(input);
        }

        public Mat getDistance(Mat src) {
            Scalar GREEN = new Scalar(0, 255, 0);

            //these values represents various of the yellow chair I need to calculate the distance
            //https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
            double KNOWN_WIDTH = 24;
            double KNOWN_DIST = 72;
            double KNOWN_PIXELS = 162;

            double MAT_SIZE = 650;

            Imgproc.resize(src, src, new Size(MAT_SIZE, MAT_SIZE));
            Mat dst = new Mat();

            Imgproc.cvtColor(dst, dst, Imgproc.COLOR_BGR2HSV);
            Imgproc.GaussianBlur(src, dst, new Size(3, 3), 100, 100);

            //adding a mask to the dst mat
            Scalar lowerHSV = new Scalar(0, 135, 112);
            Scalar upperHSV = new Scalar(67, 196, 155);
            Core.inRange(dst, lowerHSV, upperHSV, dst);

            //get the contours of the ring
            List<MatOfPoint> contours = new ArrayList<>();
            Mat heirarchy = new Mat();
            Imgproc.findContours(dst, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //draw a contour on the src image
            //Imgproc.drawContours(src, contours, -1, GREEN, 2, Imgproc.LINE_8, heirarchy, 2, new Point());

            //finds the largest bounded rect
            Rect largest = new Rect();
            for (int i = 0; i < contours.size(); i++) {
                Rect rect = Imgproc.boundingRect(contours.get(i));
                if (largest.area() < rect.area()) largest = rect;
            }
            //draws largest rect
            Imgproc.rectangle(src, largest, GREEN);

            //I am not smart enough to make this from scratch, I used a formula from the internet to calculate the distance
            //https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
            double focus = (KNOWN_PIXELS * KNOWN_DIST) / (double) KNOWN_WIDTH;
            double distance = Double.valueOf(String.format("%.1f", (KNOWN_WIDTH * focus) / largest.width));

            Imgproc.putText(src, distance + " inches", new Point(200, 450), Imgproc.FONT_HERSHEY_COMPLEX, 1, GREEN, 2);
            return src;
            //return distance;
        }

        /**
         * this function has not been tested onto the phone yet so beware
         * many adjustments have to be made in order to make this functional
         *
         * @param src
         * @return
         */
    }
}
