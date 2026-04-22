package org.firstinspires.ftc.teamcode.DECODE.ArfitactRecognization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class ArtifactDetection {
    private OpenCvWebcam openCvWebcam;
    BallsRecog pipeline;
    private static final int Cam_WIDTH = 1920;
    private static final int Cam_HEIGHT = 1080;

    public ArtifactDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        int monitorCameraView = hardwareMap.appContext.getResources().getIdentifier(
                "monitorCameraView", "id", hardwareMap.appContext.getPackageName()
        );

        openCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), monitorCameraView);

        pipeline = new BallsRecog();
        openCvWebcam.setPipeline(pipeline);

        openCvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvWebcam.startStreaming(Cam_WIDTH, Cam_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.addData("Camera initialize", "Ready");
        telemetry.update();
    }
}
