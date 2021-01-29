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
public class ChaseRing2 extends LinearOpMode
{
    RobotCV robot;
    Controller controller1;

    @Override
    public void runOpMode()
    {
        robot = new RobotCV(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        int targetX = 60;
        int targetY = 160;
        double y = 0;
        double x = 0;

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        robot.initCamera();

        waitForStart();

        while (opModeIsActive())
        {
            robot.updateObjectValues();
            double dy = targetY - robot.objectY;
            double dx = targetX - robot.objectX;

            if (Math.abs(dx) < 10){
                x = 0;
            } else {
                dx = Range.clip(dx / 500.0, -0.2, 0.2);
                x += dx;
            }
            if (Math.abs(dy) < 10){
                y = 0;
            } else {
                dy = Range.clip(dy / 500.0, -0.2, 0.2);
                y -= dy;
            }

            y = Range.clip(y, -0.6, 0.6);
            x = Range.clip(x, -0.6, 0.6);

            double h = robot.objectHeight;
            double w = robot.objectWidth;
            double r = 1.0 * w / h;

            if ( !(h > 10 && h < 45 && w > 22 && w < 65 && r > 1.2 && r < 2.5)){
                x = 0;
                y = 0;
            }

            robot.chaseObject(x, y, targetX, targetY);

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }
    }

}