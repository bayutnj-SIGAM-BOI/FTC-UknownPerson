package org.firstinspires.ftc.teamcode.AdvanceCode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTag = new ArrayList<>();

    private Telemetry telemetry;
    public static int exposure = 4;
    public static int gain = 60;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Configure AprilTag processor with optimized settings
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setNumThreads(1)
                .build();

        // Build vision portal with webcam
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(false);
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        // Set manual exposure for consistent lighting
        setManualExposure(exposure, gain);

        telemetry.addData("AprilTag", "Initialized");
        telemetry.update();
    }

    private void setManualExposure(int exposureMs, int gain) {
        // Wait for camera to be ready
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();

            // Fixed: Changed && to || in while condition
            while (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Apply manual exposure settings
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

                // Switch to manual mode if needed
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(50);
                }

                // Set exposure and gain
                exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
                Thread.sleep(20);
                gainControl.setGain(gain);
                Thread.sleep(20);

                telemetry.addData("Exposure", "%d ms", exposureMs);
                telemetry.addData("Gain", gain);

            } catch (Exception e) {
                telemetry.addData("Camera Control Error", e.getMessage());
            }
            telemetry.update();
        }
    }

    public void update() {
        if (aprilTagProcessor != null) {
            detectedTag = aprilTagProcessor.getDetections();
        }
    }

    public List<AprilTagDetection> getDetectedTag() {
        return detectedTag;
    }


    public AprilTagDetection getTagBySpesificId(int id) {
        for (AprilTagDetection detection : detectedTag) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public int getDetectedTagCount() {
        return detectedTag.size();
    }

    public boolean isTagDetected(int id) {
        return getTagBySpesificId(id) != null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}