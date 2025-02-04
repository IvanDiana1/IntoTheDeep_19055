package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    protected OpenCvWebcam cam;
    protected OpenCvPipeline pipeline;
    private final String CAMERA_NAME;
    public int width = 320, height = 240;

    public Camera(HardwareMap hwmap, String name, int w, int h, OpenCvPipeline pipeline){
        String packageName = hwmap.appContext.getPackageName();
        int id = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", packageName);
        WebcamName camera = hwmap.get(WebcamName.class, name);
        CAMERA_NAME = name;

        this.width = w;
        this.height = h;
        this.pipeline = pipeline;

        cam = OpenCvCameraFactory.getInstance().createWebcam(camera, id);
        cam.setMillisecondsPermissionTimeout(3000);
    }

    boolean cameraOK = true;
    public void initCamera(OpenCvPipeline pipeline){
        this.pipeline = pipeline;
        OpenCvCamera.AsyncCameraOpenListener listener = new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                cam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                cameraOK = false;
            }
        };

        cam.openCameraDeviceAsync(listener);
        //return cameraOK[0]; this return made no sense as the camera is opening async
    }

    public void stopWithoutClosing(){
        cam.stopStreaming();
    }

    public void stopCamera(){
        cam.stopStreaming();
        cam.closeCameraDevice();
    }

    public void setPipeline(OpenCvPipeline pipeline){
        this.pipeline = pipeline;
        cam.setPipeline(pipeline);
    }

    public OpenCvPipeline getPipeline(){
        return pipeline;
    }

    public String getName(){
        return CAMERA_NAME;
    }
}
