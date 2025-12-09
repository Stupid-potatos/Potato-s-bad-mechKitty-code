package org.firstinspires.ftc.teamcode.Potato_Assets;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import java.util.List;

/**
 * Reusable class for turning the robot to face an AprilTag using Pedro Pathing.
 * Can be used across multiple OpModes for consistent AprilTag alignment behavior.
 */
public class TurnToFaceTag {

    private Follower follower;
    private Telemetry telemetry;
    private boolean isTurning = false;

    // Calculated values from last turn attempt
    private double robotX, robotY, robotYaw;
    private double tagX, tagY;
    private double angleToTag, distance, turnNeeded;

    /**
     * Constructor for TurnToFaceTag
     * @param follower The Pedro Pathing Follower object
     * @param telemetry Telemetry object for displaying information
     */
    public TurnToFaceTag(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
    }

    /**
     * Attempts to rotate the robot to face the currently visible AprilTag.
     * Uses AprilTag localization to determine robot + tag positions,
     * computes the correct turn angle, then commands Pedro to rotate.
     *
     * @param detections List of AprilTag detections from AprilTagReader
     * @return true if turn was initiated, false if no valid tag found
     */
    public boolean executeTurn(List<AprilTagDetection> detections) {
        // Ensure a tag exists
        if (detections.isEmpty()) {
            telemetry.addLine(">>> NO TAGS DETECTED <<<");
            telemetry.update();
            return false;
        }

        // Always pick the first detected tag
        AprilTagDetection detection = detections.get(0);

        // Tag must have metadata (field position info)
        if (detection.metadata == null) {
            telemetry.addLine(">>> TAG HAS NO METADATA <<<");
            telemetry.update();
            return false;
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

        return true;
    }

    /**
     * Checks if the robot is currently turning to face a tag.
     * Should be called in your OpMode loop to monitor turn completion.
     *
     * @return true if turning is in progress, false otherwise
     */
    public boolean isTurning() {
        return isTurning;
    }

    /**
     * Updates the turning state by checking if Pedro has completed the turn.
     * Call this in your OpMode loop after follower.update().
     *
     * @return true if turn just completed this cycle, false otherwise
     */
    public boolean update() {
        if (isTurning) {
            if (!follower.isBusy()) {
                isTurning = false;
                return true; // Turn just completed
            }
        }
        return false; // Still turning or not turning
    }

    /**
     * Forcefully stops the current turn operation.
     */
    public void cancelTurn() {
        isTurning = false;
    }

    // Getter methods for calculated values (useful for debugging/telemetry)

    public double getRobotX() { return robotX; }
    public double getRobotY() { return robotY; }
    public double getRobotYaw() { return robotYaw; }
    public double getTagX() { return tagX; }
    public double getTagY() { return tagY; }
    public double getAngleToTag() { return angleToTag; }
    public double getDistance() { return distance; }
    public double getTurnNeeded() { return turnNeeded; }
}