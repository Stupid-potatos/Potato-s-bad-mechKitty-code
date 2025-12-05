package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;

import java.util.List;

@TeleOp(name = "AprilTag with Pedro")
public class AprilTag_Localization extends LinearOpMode {

    // Camera’s physical mount translation on the robot in inches
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

    // Camera orientation: yaw, pitch, roll
    private final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    // AprilTag & vision objects
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Pedro Pathing follower object
    private Follower follower;

    // Variables used for calculating turns toward a tag
    double robotX, robotY, robotYaw;
    double tagX, tagY;
    double angleToTag, distance, turnNeeded;

    // Turn state management
    private boolean isTurning = false;

    // Debounce for A button (so it only triggers once per press)
    private boolean aButtonPreviouslyPressed = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);  // Initialize Pedro follower
        initAprilTag();                                     // Start AprilTag detection

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update(); // Pedro follower update loop—must run every iteration

            telemetryAprilTag(); // Display AprilTag data
            telemetry.update();

            // --- BUTTON DEBOUNCE FOR 'A' ---
            boolean aButtonCurrentlyPressed = gamepad1.a;
            if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed && !isTurning) {
                turnToFaceTag();  // Begin turn maneuver
            }
            aButtonPreviouslyPressed = aButtonCurrentlyPressed;

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
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            sleep(20); // Reduce CPU load
        }

        visionPortal.close(); // Cleanup camera resources
    }


    /**
     * Attempts to rotate the robot to face the currently visible AprilTag.
     * Uses AprilTag localization to determine robot + tag positions,
     * computes the correct turn angle, then commands Pedro to rotate.
     */
    private void turnToFaceTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

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

        // Ignore “Obelisk” tags (these are vertical)
        if (detection.metadata.name.contains("Obelisk")) {
            telemetry.addLine(">>> SKIPPING OBELISK TAG <<<");
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
        angleToTag = Math.toDegrees(Math.atan2(tagY - robotY, tagX - robotX));

        // Compute smallest-rotation needed (-180 to +180)
        turnNeeded = angleToTag - robotYaw;
        //while (turnNeeded > 180) turnNeeded -= 360;
        //while (turnNeeded < -180) turnNeeded += 360;

        // Distance to the tag (useful for debugging or future logic)
        distance = Math.sqrt(Math.pow(tagX - robotX, 2) + Math.pow(tagY - robotY, 2));

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
     * Builds AprilTagProcessor + VisionPortal with camera calibration and resolution.
     * Ensures AprilTag pose estimations use correct extrinsics.
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(
                        700.612000761,   // fx
                        700.612000761,   // fy
                        395.22208662,    // cx
                        300.762956164    // cy
                ) // Calibration values
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280,720));
        builder.addProcessor(aprilTag);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        visionPortal = builder.build();
    }


    /**
     * Displays processed AprilTag + robot pose info on telemetry.
     * Shows robot position, heading, tag info, turn needed, and distance.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (isTurning) telemetry.addLine(">>> TURNING IN PROGRESS <<<");

        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
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

                // Skip obelisk tags for calculations
                if (!detection.metadata.name.contains("Obelisk")) {
                    tagX = detection.metadata.fieldPosition.get(0);
                    tagY = detection.metadata.fieldPosition.get(1);


                    telemetry.addLine(String.format(
                            "bot XY heading %6.0f %6.0f %6.0f ",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                    // Recompute values for telemetry display
                    angleToTag = Math.toDegrees(Math.atan2(tagY - robotY, tagX - robotX));
                    turnNeeded = angleToTag - robotYaw;
                    while (turnNeeded > 180) turnNeeded -= 360;
                    while (turnNeeded < -180) turnNeeded += 360;

                    distance = Math.sqrt(Math.pow(tagX - robotX, 2) + Math.pow(tagY - robotY, 2));

                    telemetry.addLine(String.format("Robot Heading %6.1f  (deg)", robotYaw));
                    telemetry.addLine(String.format("Tag XY %6.1f %6.1f  (inch)", tagX, tagY));
                    telemetry.addLine(String.format("Turn needed: %6.1f degrees", turnNeeded));
                    telemetry.addLine(String.format("Distance: %6.1f inches", distance));
                }

            } else {
                // Tags without metadata (shouldn’t happen on CENTERSTAGE field)
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(
                        "bot XY heading %6.0f %6.0f %6.0f ",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)

                ));
            }
        }

        telemetry.addLine("\nPress A to turn and face tag");
        telemetry.addLine("D-pad Up: Resume camera | Down: Stop camera");

    }
}
