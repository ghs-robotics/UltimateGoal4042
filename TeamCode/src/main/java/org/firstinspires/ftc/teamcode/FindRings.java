/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
public class FindRings extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    RingDeterminationPipeline pipeline;


    @Override
    public void runOpMode()
    {
        int ringX = 0;
        int ringY = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            ringX = RingDeterminationPipeline.ringX;
            ringY = RingDeterminationPipeline.ringY;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("ringX, ringY", "( " + ringX + ", " + ringY + " )");
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        public Mat mask;
        public static int ringX = 0;
        public static int ringY = 0;

        /*
         * An enum to define the ring position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            //update ring coordinates
            int[] coords = getRingCoordinates(input);
            ringX = coords[0];
            ringY = coords[1];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    new Point(coords[0],coords[1]), // First point which defines the rectangle
                    new Point(coords[0] + coords[2],coords[1] + coords[3]), // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return mask;
        }

        public int getAnalysis()
        {
            return avg1;
        }

        public int[] getRingCoordinates(Mat input) {
            Scalar GREEN = new Scalar(0, 255, 0);

            Mat src = input;
            Imgproc.resize(src, src, new Size(320, 240));
            Mat dst = new Mat();

            Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

            //adding a mask to the dst mat
            Scalar lowerHSV = new Scalar(50, 100, 0); //74, 153, 144
            Scalar upperHSV = new Scalar(200, 255, 255); //112, 242, 255
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

            mask = dst;
            return new int[]{largest.x,largest.y, largest.width, largest.height};
        }
    }

}