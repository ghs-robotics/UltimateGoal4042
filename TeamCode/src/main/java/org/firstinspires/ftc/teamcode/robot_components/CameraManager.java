package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CameraManager {

    // CAUTION, I changed the type of the camera to OpenCvCamera so I can make the cameras interchangeable
//    public OpenCvInternalCamera phoneCam;
    public OpenCvCamera phoneCam;
    public OpenCvCamera webcam;

    // Current camera alternates between the phoneCam and the webcam
//    public OpenCvCamera currentCamera;
    public ObjectDeterminationPipeline phoneCamPipeline;
    public ObjectDeterminationPipeline webcamPipeline;

    public CameraManager(HardwareMap hardwareMap) {
        // Initializing some CV variables/objects
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
                hardwareMap.get(WebcamName.class,"webcam"), viewportContainerIds[1]);

        phoneCam.openCameraDevice();
        webcam.openCameraDevice();

        phoneCamPipeline = new ObjectDeterminationPipeline();
        webcamPipeline = new ObjectDeterminationPipeline();

        phoneCam.setPipeline(phoneCamPipeline);
        webcam.setPipeline(webcamPipeline);

        // The phone camera is the default camera
//        currentCamera = phoneCam;
    }

    // Initialize the camera
    public void initCamera() {
        startStreaming();
        /*
        // Sets the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        currentCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        currentCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                startStreaming();
            }
        });
        */
    }

    //Start streaming frames on the phone camera
    public void startStreaming() {
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    //Stop streaming frames on the phone camera
    public void stopStreaming() {
        phoneCam.stopStreaming();
        webcam.stopStreaming();
    }

    public int[] getObjectData(String target) {
        if (target.equals("tower")) {
            return webcamPipeline.getObjectData();
        }
        return phoneCamPipeline.getObjectData();
    }
}