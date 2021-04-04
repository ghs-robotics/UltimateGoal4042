package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CameraManager implements HSVConstants {

    // Cameras
    public OpenCvCamera phoneCam;
    public OpenCvCamera webcam;

    // Pipelines for image processing
    public CVDetectionPipeline phoneCamPipeline;
    public CVDetectionPipeline webcamPipeline;

    // Constructs a CameraManager with two cameras
    public CameraManager(HardwareMap hardwareMap) {

        // Initializes some CV variables/objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, // The container we're splitting
                        2, // The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), viewportContainerIds[1]);

        // Opens cameras
        phoneCam.openCameraDevice();
        webcam.openCameraDevice();

        // Creates and assigns each camera a pipeline
        phoneCamPipeline = new CVDetectionPipeline();
        webcamPipeline = new CVDetectionPipeline();

        phoneCam.setPipeline(phoneCamPipeline);
        webcam.setPipeline(webcamPipeline);
    }

    // Returns the coordinates of the target object using CV
    public int[] getObjectData(String target) {
        if (target.equals("tower")) {
            return webcamPipeline.getObjectData();
        }
        return phoneCamPipeline.getObjectData();
    }

    // Initializes the camera
    public void initCamera() {
        startStreaming();
    }

    // Starts streaming frames on the phone camera
    public void startStreaming() {
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    // Stops streaming frames on the phone camera
    public void stopStreaming() {
        phoneCam.stopStreaming();
        webcam.stopStreaming();
    }
}