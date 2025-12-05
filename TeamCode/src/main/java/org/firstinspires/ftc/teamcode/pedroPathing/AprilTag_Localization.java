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
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Follower follower;

    // Variables for AprilTag alignment
    double robotX;
    double robotY;
    double robotYaw;
    double tagX;
    double tagY;
    double angleToTag;
    double distance;
    double turnNeeded;

    private boolean isTurning = false;
    private boolean aButtonPreviouslyPressed = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();  // Always update follower

            // Update telemetry with current AprilTag info
            telemetryAprilTag();
            telemetry.update();

            // Button debouncing: only trigger on button press (not hold)
            boolean aButtonCurrentlyPressed = gamepad1.a;
            if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed && !isTurning) {
                turnToFaceTag();
            }
            aButtonPreviouslyPressed = aButtonCurrentlyPressed;

            // Check if turn is complete
            if (isTurning) {
                if (!follower.isBusy()) {
                    isTurning = false;
                    telemetry.addLine(">>> TURN COMPLETE <<<");
                    telemetry.update();
                } else {
                    telemetry.addLine(">>> TURNING... <<<");
                }
            }

            // Camera controls
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            sleep(20);
        }

        visionPortal.close();
    }

    private void turnToFaceTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.isEmpty()) {
            telemetry.addLine(">>> NO TAGS DETECTED <<<");
            telemetry.update();
            return;
        }

        AprilTagDetection detection = currentDetections.get(0);

        if (detection.metadata == null) {
            telemetry.addLine(">>> TAG HAS NO METADATA <<<");
            telemetry.update();
            return;
        }

        if (detection.metadata.name.contains("Obelisk")) {
            telemetry.addLine(">>> SKIPPING OBELISK TAG <<<");
            telemetry.update();
            return;
        }

        // Get robot position from AprilTag localization
        robotX = detection.robotPose.getPosition().x;
        robotY = detection.robotPose.getPosition().y;
        robotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Get tag position from field metadata
        tagX = detection.metadata.fieldPosition.get(0);   /// THIS COULD BE REVERSED, COULD EXPLAIN THE DIAGONAL MOVEMENT
        tagY = detection.metadata.fieldPosition.get(1);   /// THIS COULD BE REVERSED, COULD EXPLAIN THE DIAGONAL MOVEMENT


        // Calculate angle to tag
        angleToTag = Math.toDegrees(Math.atan2(tagY - robotY, tagX - robotX));

        // Calculate turn needed (normalize to -180 to 180)
        turnNeeded = angleToTag - robotYaw;
        while (turnNeeded > 180) turnNeeded -= 360;
        while (turnNeeded < -180) turnNeeded += 360;

        // Calculate distance
        distance = Math.sqrt(Math.pow(tagX - robotX, 2) + Math.pow(tagY - robotY, 2));

        // Update Pedro's pose to match AprilTag localization
        Pose currentPose = new Pose(robotX, robotY, Math.toRadians(robotYaw));
        follower.setPose(currentPose);

        Pose targetPose = new Pose(robotX,robotY, Math.toRadians(angleToTag));

        // Command the turn using Pedro Pathing
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), targetPose))
                        .setConstantHeadingInterpolation(Math.toRadians(angleToTag))
                        .build(),
                true
        );

        isTurning = true;

        telemetry.addLine(">>> STARTING TURN <<<");
        telemetry.addData("Current Heading", "%.1f deg", robotYaw);
        telemetry.addData("Target Heading", "%.1f deg", angleToTag);
        telemetry.addData("Turn Needed", "%.1f deg", turnNeeded);
        telemetry.addData("Distance to Tag", "%.1f inches", distance);
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        ///  MAYBE NEED CALIBRATION ON CAMERA

        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (isTurning) {
            telemetry.addLine(">>> TURNING IN PROGRESS <<<");
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));

                if (!detection.metadata.name.contains("Obelisk")) {
                    // Update values for telemetry
                    robotX = detection.robotPose.getPosition().x;
                    robotY = detection.robotPose.getPosition().y;
                    robotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    tagX = detection.metadata.fieldPosition.get(0);
                    tagY = detection.metadata.fieldPosition.get(1);

                    angleToTag = Math.toDegrees(Math.atan2(tagY - robotY, tagX - robotX));
                    turnNeeded = angleToTag - robotYaw;
                    while (turnNeeded > 180) turnNeeded -= 360;
                    while (turnNeeded < -180) turnNeeded += 360;

                    distance = Math.sqrt(Math.pow(tagX - robotX, 2) + Math.pow(tagY - robotY, 2));

                    telemetry.addLine(String.format("Robot XYZ %6.1f %6.1f %6.1f  (inch)",
                            robotX, robotY, detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("Robot Heading %6.1f  (deg)", robotYaw));
                    telemetry.addLine(String.format("Tag XY %6.1f %6.1f  (inch)", tagX, tagY));
                    telemetry.addLine(String.format("Turn needed: %6.1f degrees", turnNeeded));
                    telemetry.addLine(String.format("Distance: %6.1f inches", distance));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nPress A to turn and face tag");
        telemetry.addLine("D-pad Up: Resume camera | Down: Stop camera");
    }
}