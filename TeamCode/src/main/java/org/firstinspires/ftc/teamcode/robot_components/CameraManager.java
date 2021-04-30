package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.data.HSVConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.concurrent.CopyOnWriteArrayList;

public class CameraManager implements HSVConstants {

    private int cameraMonitorViewId;
    private boolean streaming = false;

    // Cameras
    public OpenCvCamera phoneCam;
    public OpenCvCamera webcam;

    // Pipelines for image processing
    public CVDetectionPipeline phoneCamPipeline;
    public CVDetectionPipeline webcamPipeline;

    HardwareMap hardwareMap;
    ElapsedTime elapsedTime;

    // Constructs a CameraManager with two cameras
    public CameraManager(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        elapsedTime = new ElapsedTime();

        // Initializes some CV variables/objects
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());


        // Uncomment ONE of the below statements ONLY
//        setUpDualPort(hardwareMap);
//        setUpSinglePhonePort(hardwareMap);
        setUpSingleWebcamPort(hardwareMap);
//        setUpNoStreamPort(hardwareMap);

        // Opens cameras
        phoneCam.openCameraDevice();
        webcam.openCameraDevice();

        // Creates and assigns each camera a pipeline
        phoneCamPipeline = new CVDetectionPipeline();
        webcamPipeline = new CVDetectionPipeline();

        phoneCamPipeline.activeObjects = new CopyOnWriteArrayList<>(); // Initializes the ArrayLists in each pipeline
        webcamPipeline.activeObjects = new CopyOnWriteArrayList<>();

        phoneCam.setPipeline(phoneCamPipeline);
        webcam.setPipeline(webcamPipeline);
    }

    // Initializes the camera
    public void initCamera() {
        startStreaming();
    }

    public boolean isStreaming() {
        return streaming;
    }

    public boolean isWebcamReady() {
        return elapsedTime.seconds() > 1.7;
    }

    private void setUpDualPort(HardwareMap hardwareMap) {
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, // The container we're splitting
                        2, // The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), viewportContainerIds[1]);
    }

    private void setUpNoStreamPort(HardwareMap hardwareMap) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"));
    }

    private void setUpSinglePhonePort(HardwareMap hardwareMap) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"));
    }

    private void setUpSingleWebcamPort(HardwareMap hardwareMap) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
    }

    // Starts streaming frames on the phone camera
    public void startStreaming() {
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        streaming = true;
        elapsedTime.reset();
    }

    // Stream in the middle of a match
    public void startMidStream() {
        if (!streaming) {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, "Webcam 1"));
            webcam.setPipeline(webcamPipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
            webcam.openCameraDevice();
            streaming = true;
            elapsedTime.reset();
        }
    }

    // Stops streaming frames on the phone camera
    public void stopStreaming() {
        streaming = false;
        phoneCam.stopStreaming();
        webcam.stopStreaming();
    }
}