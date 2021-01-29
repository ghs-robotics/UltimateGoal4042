package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This will be the main class that we will put all of our autonomous code into
 * This class is obviously not complete, and the structure of the code is not decided yet
 */
@Autonomous
public class Auto1 extends LinearOpMode {

    private OpenCvInternalCamera phoneCam;
    private OpenCVProcess pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new OpenCVProcess();
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

        if (opModeIsActive()) {

            //move towards the desired target wobble goal zone

            //once there, place down the wobble goal

            //aim the robot at the goal and make sure that the robot is within the launch zone

            //shoot 3 shots

            //pickup 3 rings

            telemetry.addData("opmode is working","");
            telemetry.update();
        }
    }

    public static class OpenCVProcess extends OpenCvPipeline {

        @Override
        public void init(Mat firstFrame) {
            //start opencv program to count rings
        }

        @Override
        public Mat processFrame(Mat input) {
            //run several opencv programs to change variables and stuff
            return input;
        }
    }
}
