package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assets.AprilTagReader;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import org.firstinspires.ftc.teamcode.Potato_Assets.TurnToFaceTag;

@TeleOp(name = "AprilTag with Pedro")
public class AprilTag_Localization extends LinearOpMode {

    // AprilTag reader instance
    private AprilTagReader aprilTagReader;

    // TurnToFaceTag utility
    private TurnToFaceTag turnToFaceTag;

    // Debounce for X button (so it only triggers once per press)
    private boolean xButtonPreviouslyPressed = false;

    @Override
    public void runOpMode() {
        // Pedro Pathing follower object
        Follower follower = Constants.createFollower(hardwareMap);  // Initialize Pedro follower
        aprilTagReader = new AprilTagReader(hardwareMap);  // Start AprilTag detection
        turnToFaceTag = new TurnToFaceTag(follower, telemetry); // Initialize turn utility

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update(); // Pedro follower update loop—must run every iteration

            telemetryAprilTag(); // Display AprilTag data
            telemetry.update();

            // --- BUTTON DEBOUNCE FOR 'X' ---
            boolean xButtonCurrentlyPressed = gamepad1.x;
            if (xButtonCurrentlyPressed && !xButtonPreviouslyPressed && !turnToFaceTag.isTurning()) {
                // Get current detections and attempt to turn
                List<AprilTagDetection> detections = aprilTagReader.getFilteredDetections();
                turnToFaceTag.executeTurn(detections);
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // --- TURN COMPLETION CHECK ---
            if (turnToFaceTag.update()) {
                // Turn just completed
                telemetry.addLine(">>> TURN COMPLETE <<<");
                telemetry.update();
            } else if (turnToFaceTag.isTurning()) {
                telemetry.addLine(">>> TURNING... <<<");
            }

            // --- CAMERA STREAM CONTROLS ---
            if (gamepad1.dpad_down) {
                aprilTagReader.stopStreaming();
            } else if (gamepad1.dpad_up) {
                aprilTagReader.resumeStreaming();
            }

            sleep(20); // Reduce CPU load
        }

        aprilTagReader.close(); // Cleanup camera resources
    }

    /**
     * Displays processed AprilTag + robot pose info on telemetry.
     * Shows robot position, heading, tag info, turn needed, and distance.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        // Use AprilTagReader's getFilteredDetections() method
        List<AprilTagDetection> currentDetections = aprilTagReader.getFilteredDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addData("Camera Streaming", aprilTagReader.isStreaming() ? "Yes" : "No");

        if (turnToFaceTag.isTurning()) {
            telemetry.addLine(">>> TURNING IN PROGRESS <<<");
        }

        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format(
                    "\n==== (ID %d) %s",
                    detection.id,
                    detection.metadata != null ? detection.metadata.name : "Unknown"
            ));

            // Extract robot pose from this tag
            double robotX = detection.robotPose.getPosition().x;
            double robotY = detection.robotPose.getPosition().y;
            double robotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addLine(String.format(
                    "Robot XY %6.1f %6.1f %6.1f  (inch)",
                    robotX, robotY, robotYaw
            ));

            if (detection.metadata != null) {
                // Get tag position from metadata
                double tagX = detection.metadata.fieldPosition.get(0);
                double tagY = detection.metadata.fieldPosition.get(1);

                // Recompute values for telemetry display
                double angleToTag = detection.ftcPose.bearing;
                double turnNeeded = angleToTag - robotYaw;
                double distance = detection.ftcPose.range;

                telemetry.addLine(String.format("Robot Heading %6.1f  (deg)", robotYaw));
                telemetry.addLine(String.format("Tag XY %6.1f %6.1f  (inch)", tagX, tagY));
                telemetry.addLine(String.format("Turn needed: %6.1f degrees", turnNeeded));
                telemetry.addLine(String.format("Distance: %6.1f inches", distance));
            }
        }

        telemetry.addLine("\nPress X to turn and face tag");
        telemetry.addLine("D-pad Up: Resume camera | Down: Stop camera");
    }
}