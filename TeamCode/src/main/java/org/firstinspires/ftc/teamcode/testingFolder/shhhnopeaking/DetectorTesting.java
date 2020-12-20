package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectorTesting {
    private OpenCvCamera camera;
    private boolean isUsingWebcam;
    private String webcamName;
    private HardwareMap hardwareMap;
//    private UGRectRingPipeline ftclibPipeline;
    private PipelineTesting pipelineTesting;
    private BeautifulPipeline popeop;
    //The constructor is overloaded to allow the use of webcam instead of the phone camera
    public DetectorTesting(HardwareMap hMap) {
        hardwareMap = hMap;
    }

    public DetectorTesting(HardwareMap hMap, String webcamName) {
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        if (isUsingWebcam) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        } else {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        //Set the pipeline the camera should use and start streaming
        camera.setPipeline(pipelineTesting = new PipelineTesting());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public Stack getStack() {
        if(popeop.getAvg() < 15)
            return Stack.ZERO;
        else
            return Stack.FOUR;
    }

    public enum Stack {
        ZERO,
        ONE,
        FOUR,
    }

}

