package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assests.AprilTagReader;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name = "AprilTag with Pedro")
public class AprilTag_Localization extends LinearOpMode {

    // Pedro Pathing follower object
    private Follower follower;

    // AprilTag reader instance
    private AprilTagReader aprilTagReader;

    // Variables used for calculating turns toward a tag
    double robotX, robotY, robotYaw;
    double tagX, tagY;
    double angleToTag, distance, turnNeeded;

    // Turn state management
    private boolean isTurning = false;

    // Debounce for A button (so it only triggers once per press)
    private boolean xButtonPreviouslyPressed = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);  // Initialize Pedro follower
        aprilTagReader = new AprilTagReader(hardwareMap);  // Start AprilTag detection

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update(); // Pedro follower update loop—must run every iteration

            telemetryAprilTag(); // Display AprilTag data
            telemetry.update();

            // --- BUTTON DEBOUNCE FOR 'A' ---
            boolean xButtonCurrentlyPressed = gamepad1.x;
            if (xButtonCurrentlyPressed && !xButtonPreviouslyPressed && !isTurning) {
                turnToFaceTag();  // Begin turn maneuver
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // --- TURN COMPLETION CHECK ---
            if (isTurning) {
                if (!follower.isBusy()) {  // Pedro finished moving
                    isTurning = false;
                    telemetry.addLine(">>> TURN COMPLETE <<<");
                    telemetry.update();
                } else {
                    telemetry.addLine(">>> TURNING... <<<");
                }
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
     * Attempts to rotate the robot to face the currently visible AprilTag.
     * Uses AprilTag localization to determine robot + tag positions,
     * computes the correct turn angle, then commands Pedro to rotate.
     */
    private void turnToFaceTag() {
        // Use AprilTagReader's getFilteredDetections() method
        List<AprilTagDetection> currentDetections = aprilTagReader.getFilteredDetections();

        // Ensure a tag exists
        if (currentDetections.isEmpty()) {
            telemetry.addLine(">>> NO TAGS DETECTED <<<");
            telemetry.update();
            return;
        }

        // Always pick the first detected tag
        AprilTagDetection detection = currentDetections.get(0);

        // Tag must have metadata (field position info)
        if (detection.metadata == null) {
            telemetry.addLine(">>> TAG HAS NO METADATA <<<");
            telemetry.update();
            return;
        }

        // Extract robot pose (as determined from AprilTag)
        robotX = detection.robotPose.getPosition().x;
        robotY = detection.robotPose.getPosition().y;
        robotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Field-defined tag position
        tagX = detection.metadata.fieldPosition.get(0);
        tagY = detection.metadata.fieldPosition.get(1);

        // Compute angle from robot to tag
        angleToTag = detection.ftcPose.bearing;

        // Compute smallest-rotation needed (-180 to +180)
        turnNeeded = angleToTag - robotYaw;

        // Distance to the tag (useful for debugging or future logic)
        distance = detection.ftcPose.range;

        // Update Pedro's internal pose to match AprilTag-derived pose
        Pose currentPose = new Pose(robotX, robotY, Math.toRadians(robotYaw));
        follower.setPose(currentPose);

        // Build a target pose with the same X/Y but rotated to face the tag
        Pose targetPose = new Pose(robotX, robotY, Math.toRadians(angleToTag));

        // Build a simple Bezier-line path to rotate robot
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), targetPose.getPose()))
                        .setLinearHeadingInterpolation(
                                currentPose.getHeading(),
                                targetPose.getHeading()
                        )
                        .build()
        );

        isTurning = true;

        // Telemetry feedback
        telemetry.addLine(">>> STARTING TURN <<<");
        telemetry.addData("Current Heading", "%.1f deg", robotYaw);
        telemetry.addData("Target Heading", "%.1f deg", angleToTag);
        telemetry.addData("Turn Needed", "%.1f deg", turnNeeded);
        telemetry.addData("Distance to Tag", "%.1f inches", distance);
        telemetry.update();
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

        if (isTurning) telemetry.addLine(">>> TURNING IN PROGRESS <<<");

        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format(
                    "\n==== (ID %d) %s",
                    detection.id,
                    detection.metadata.name
            ));

            // Extract robot pose from this tag
            robotX = detection.robotPose.getPosition().x;
            robotY = detection.robotPose.getPosition().y;
            robotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addLine(String.format(
                    "Robot XY %6.1f %6.1f %6.1f  (inch)",
                    robotX, robotY, robotYaw
            ));

            // Get tag position from metadata
            tagX = detection.metadata.fieldPosition.get(0);
            tagY = detection.metadata.fieldPosition.get(1);

            // Recompute values for telemetry display
            angleToTag = detection.ftcPose.bearing;
            turnNeeded = angleToTag - robotYaw;

            distance = detection.ftcPose.range;

            telemetry.addLine(String.format("Robot Heading %6.1f  (deg)", robotYaw));
            telemetry.addLine(String.format("Tag XY %6.1f %6.1f  (inch)", tagX, tagY));
            telemetry.addLine(String.format("Turn needed: %6.1f degrees", turnNeeded));
            telemetry.addLine(String.format("Distance: %6.1f inches", distance));
        }

        telemetry.addLine("\nPress A to turn and face tag");
        telemetry.addLine("D-pad Up: Resume camera | Down: Stop camera");
    }
}