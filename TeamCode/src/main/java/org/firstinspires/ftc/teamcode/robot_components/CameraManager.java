package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CameraManager {

    //CAUTION, I changed the type of the camera to OpenCvCamera so I can make the camera's interchangable
    //public OpenCvInternalCamera phonecam;
    public OpenCvCamera phonecam;
    public OpenCvCamera webcam;
    public OpenCvCamera currentCamera;
    public ObjectDeterminationPipeline pipeline;

    public int[] getObjectValues() {
        return  new int[]{1,2,3,4};
    }

    public CameraManager(HardwareMap hardwareMap) {
        // Initializing some CV variables/objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        phonecam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"webcam"), viewportContainerIds[1]);
        phonecam.setPipeline(pipeline);
        webcam.setPipeline(pipeline);
    }

    //Initialize the camera
    public void initCamera() {
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
    }

    void initWebcam() {
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                startStreaming();
            }
        });
    }

    //Start streaming frames on the phone camera
    public void startStreaming() {
        phonecam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    //Stop streaming frames on the phone camera
    public void stopStreaming() {
        phonecam.stopStreaming();
    }
}
