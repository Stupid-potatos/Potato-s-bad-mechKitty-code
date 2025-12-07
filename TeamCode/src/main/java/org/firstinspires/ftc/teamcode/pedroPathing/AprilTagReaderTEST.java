package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Potato_Assests.AprilTagReader;

@TeleOp(name = "AprilTag Data Reader")
public class AprilTagReaderTEST extends LinearOpMode {
    private AprilTagReader aprilTagReader;

    @Override
    public void runOpMode() {
        // Initialize the AprilTag reader
        aprilTagReader = new AprilTagReader(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.backWasReleased()) {
                aprilTagReader.stopStreaming();
            } else if (gamepad1.xWasPressed()) {
                aprilTagReader.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        aprilTagReader.close();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        int detectionCount = aprilTagReader.getDetectionCount();
        telemetry.addData("# AprilTags Detected", detectionCount);

        // Get all filtered detections
        for (AprilTagDetection detection : aprilTagReader.getFilteredDetections()) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id,
                    detection.metadata != null ? detection.metadata.name : "Unknown"));

            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            telemetry.addLine(String.format("TAG RBE %6.1f %6.1f %6.1f",
                    detection.ftcPose.range,             // Distance from tag
                    detection.ftcPose.bearing,           // Angle between robot heading and tag
                    detection.ftcPose.elevation));       // Vertical angle from camera center line to the tag
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addData("Streaming", aprilTagReader.isStreaming() ? "Yes" : "No");
    }
}