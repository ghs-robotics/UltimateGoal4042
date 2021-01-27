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
import com.qualcomm.robotcore.util.Range;

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
public class ChaseWobbleGoal extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    RingDeterminationPipeline pipeline;

    Robot robot;
    Controller controller1;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        int towerX = 0;
        int towerY = 0;
        int towerWidth = 0;
        int towerHeight = 0;
        int targetX = 95;
        int targetWidth = 75;
        int shots = 0;
        double y = 0;
        double x = 0;

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
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        robot.resetServos();

        waitForStart();
        robot.resetElapsedTime();

        while (opModeIsActive())
        {
            towerX = RingDeterminationPipeline.wobbleX;
            towerY = RingDeterminationPipeline.wobbleY;
            towerWidth = RingDeterminationPipeline.wobbleWidth;
            towerHeight = RingDeterminationPipeline.wobbleHeight;

            double dw = targetWidth - towerWidth;
            double dx = targetX - towerX;

            if (Math.abs(dx) < 3) {
                x = 0;
            } else {
                dx = Range.clip(dx / 250.0, -0.2, 0.2);
                x += dx;
            }
            if (Math.abs(dw) < 3) {
                y = 0;
            } else {
                dw = Range.clip(dw / 140.0, -0.2, 0.2);
                y -= dw;
            }

            y = Range.clip(y, -0.6, 0.6);
            x = Range.clip(x, -0.6, 0.6);

            if (!(towerWidth > 60 && towerWidth < 220)) {
                x = 0;
                y = 0;
            }
            robot.startMoving(x, y, towerX, towerY, towerWidth, towerHeight, targetX, targetWidth);
            if (robot.elapsedTime.seconds() > 6 && shots < 3){
                robot.startMoving(0, 0, towerX, towerY, towerWidth, towerHeight, targetX, targetWidth);
                robot.toggleShooter();
                robot.wait(1.5);
                robot.launchRing();
                robot.wait(0.5);
                robot.launchRing();
                robot.wait(0.5);
                robot.launchRing();
                shots += 3;
                robot.toggleShooter();
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        public Mat mask;
        public static int wobbleX = 0;
        public static int wobbleY = 0;
        public static int wobbleWidth = 0;
        public static int wobbleHeight = 0;

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 123, 0);
        static final Scalar GREEN = new Scalar(28, 255,88);

        @Override
        public void init(Mat firstFrame)
        {
        }

        @Override
        public Mat processFrame(Mat input)
        {

            //update ring coordinates
            int[] coords = getWobbleCoordinates(input);
            wobbleX = coords[0];
            wobbleY = coords[1];
            wobbleWidth = coords[2];
            wobbleHeight = coords[3];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    new Point(wobbleX, wobbleY), // First point which defines the rectangle
                    new Point(wobbleX + wobbleWidth, wobbleY + wobbleHeight), // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int[] getWobbleCoordinates(Mat input) {
            Scalar GREEN = new Scalar(0, 255, 0);

            Mat src = input;
            Imgproc.resize(src, src, new Size(320, 240));
            Mat dst = new Mat();

            Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);
            Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

            //adding a mask to the dst mat
            Scalar lowerHSV = new Scalar(0, 0, 0);
            Scalar upperHSV = new Scalar(255, 255, 10);
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