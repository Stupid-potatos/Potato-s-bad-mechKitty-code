package org.firstinspires.ftc.teamcode.Potato_Assets;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * AprilTagReader - Handles AprilTag detection and camera initialization
 * Provides easy access to tag detection data for navigation
 */
public class AprilTagReader {
    private AprilTagProcessor aprilTag;           // AprilTag processor instance
    private VisionPortal visionPortal;            // Camera vision portal
    private final HardwareMap hardwareMap;        // Robot hardware map

    /**
     * Constructor - Initializes AprilTag detection system
     * @param hardwareMap Robot hardware map for camera access
     */
    public AprilTagReader(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initAprilTag();
    }

    /**
     * Initialize AprilTag processor and camera setup
     * Uses constants from CameraConstants class
     */
    private void initAprilTag() {
        // Create AprilTag processor with configuration
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)    // Draw outlines on detected tags
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)  // Tag family
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)   // Output units
                .setCameraPose(CameraConstants.CAMERA_POSITION, CameraConstants.CAMERA_ORIENTATION)
                .setLensIntrinsics(
                        CameraConstants.CAMERA_FX,
                        CameraConstants.CAMERA_FY,
                        CameraConstants.CAMERA_CX,
                        CameraConstants.CAMERA_CY)
                .build();

        aprilTag.setDecimation((float) CameraConstants.DECIMATION);  // Set detection decimation

        // Setup VisionPortal for camera access
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, CameraConstants.CAMERA_NAME));                   // Camera hardware
        builder.setCameraResolution(new Size(CameraConstants.CAMERA_WIDTH, CameraConstants.CAMERA_HEIGHT));  // Resolution
        builder.enableLiveView(true);                              // Enable driver station preview
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);  // Use MJPEG format
        builder.addProcessor(aprilTag);                            // Add AprilTag processor

        visionPortal = builder.build();     // Build and start vision portal
    }

    // ===== Detection Access Methods =====

    /**
     * Get all current AprilTag detections (including filtered ones)
     * @return List of all detected AprilTags
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Get filtered detections (excludes tags matching TAG_FILTER)
     * @return Filtered list of AprilTag detections
     */
    public List<AprilTagDetection> getFilteredDetections() {
        List<AprilTagDetection> allDetections = aprilTag.getDetections();
        // Remove tags that contain the filter keyword
        allDetections.removeIf(detection ->
                detection.metadata != null &&
                        detection.metadata.name.contains(CameraConstants.TAG_FILTER)
        );
        return allDetections;
    }

    /**
     * Get count of currently detected tags
     * @return Number of detected tags
     */
    public int getDetectionCount() {
        return aprilTag.getDetections().size();
    }

    // ===== Camera Control Methods =====

    /**
     * Stop camera streaming to save CPU resources
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Resume camera streaming
     */
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Check if camera is currently streaming
     * @return true if streaming, false otherwise
     */
    public boolean isStreaming() {
        return visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Close vision portal and release camera resources
     * Call this when done with AprilTag detection
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}