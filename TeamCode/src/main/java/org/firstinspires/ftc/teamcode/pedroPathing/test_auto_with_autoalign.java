package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assets.Config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BLUE Test Auto", group = "Potato's testing")
public class test_auto_with_autoalign extends OpMode {
    // ============================================
    // APRILTAG LOCALIZATION
    // ============================================
    private AprilTagProcessor aprilTPR;
    private VisionPortal visionPortal;
    private double hallucination;
    private boolean is_blue_alliance = true;  // Set to false for red alliance

    // ============================================
    // MOTOR AND SERVO DECLARATIONS
    // ============================================

    // Flywheel motors - for shooting game elements
    private DcMotorEx flywheel;
    private DcMotorEx flywheel1;

    // Intake motor - for collecting game elements
    private DcMotorEx intake;
    boolean intakeIsOn = false;  // Tracks if intake is currently running

    // Timer for controlling flicker servo timing
    private ElapsedTime flickerTimer = new ElapsedTime();
    private boolean flickerReloading = false;  // Tracks if flicker is in the middle of a shooting sequence

    // Shooting system state
    boolean shootingIsOn;  // True when flywheels are spinning for shooting
    int shotsFired = 0;    // Counts how many shots have been fired in current cycle
    boolean isAligned = false;  // Tracks if robot is aligned to goal
    boolean needsAlignment = false;  // Tracks if we need to align before shooting

    // ============================================
    // SERVO DECLARATIONS
    // ============================================
    private Servo flicker;  // Servo that physically flicks/shoots game elements
    private Servo hood;     // Servo that adjusts shooting angle (if applicable)

    // ============================================
    // PATH FOLLOWING SYSTEM
    // ============================================
    private Follower follower;  // PedroPathing follower for autonomous movement

    // Timers for various timing functions
    private Timer pathTimer;    // Times how long we've been in current path state
    private Timer shootTimer;   // Controls timing between shots
    private Timer opmodeTimer;  // Tracks total opmode runtime
    private Timer localizationTimer;  // Controls how often we localize with AprilTags
    private Timer alignmentTimer;  // Times alignment attempts

    private int pathState;  // Current state in our autonomous state machine

    // ============================================
    // POSITION DEFINITIONS
    // ============================================
    private final Pose startPose = Config.BLUE_NEAR_GOAL_START; // Starting position on field
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Position to shoot from
    private final Pose pickup1Pose = new Pose(14.415827338129493, 83.91223021582736, Math.toRadians(180)); // First pickup location
    private final Pose pickup2Pose = new Pose(7.551079136690649, 59.3741007194245, Math.toRadians(180)); // Second pickup location
    private final Pose pickup3Pose = new Pose(9.945323741007194, 35.84460431654675, Math.toRadians(180)); // Third pickup location

    // ============================================
    // PATH DEFINITIONS
    // ============================================
    private Path scorePreload;  // Path from start to scoring position
    private PathChain grabPickup1, scorePickup1;  // First pickup cycle paths
    private PathChain grabPickup2, scorePickup2;  // Second pickup cycle paths
    private PathChain grabPickup3, scorePickup3;  // Third pickup cycle paths

    /**
     * Initializes AprilTag detection system for localization.
     */
    private void initializeAprilTag() {
        // Create AprilTag processor
        aprilTPR = new AprilTagProcessor.Builder()
                .build();

        // Create vision portal with AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // Adjust camera name as needed
                .addProcessor(aprilTPR)
                .build();
    }

    /**
     * Localizes the robot position using AprilTag detection.
     * Updates the follower's pose estimate based on detected tags.
     * Call this periodically during autonomous to improve localization accuracy.
     */
    public void localizeViaAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTPR.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Check for tag ID 24 (adjust based on your field setup)
            if (detection.id == 24 && !is_blue_alliance) {
                hallucination = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Validate heading is in expected range
                if (hallucination < 90 && hallucination > 0) {
                    // Convert AprilTag pose to Pedro Pathing pose
                    double x = detection.robotPose.getPosition().x;  // Already in MM from AprilTag
                    double y = detection.robotPose.getPosition().y;
                    double heading = Math.toRadians(translateTCHeading(true, hallucination));

                    // Update follower's pose estimate
                    follower.setPose(new Pose(x, y, heading));

                    telemetry.addLine("Localized via AprilTag 24!");
                }
            }

            // Check for tag ID 20 (adjust based on your field setup)
            if (detection.id == 20 && is_blue_alliance) {
                hallucination = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Validate heading is in expected range
                if (hallucination < 0 && hallucination > -90) {
                    // Convert AprilTag pose to Pedro Pathing pose
                    double x = detection.robotPose.getPosition().x;  // Already in MM from AprilTag
                    double y = detection.robotPose.getPosition().y;
                    double heading = Math.toRadians(translateTCHeading(false, hallucination));

                    // Update follower's pose estimate
                    follower.setPose(new Pose(x, y, heading));

                    telemetry.addLine("Localized via AprilTag 20!");
                }
            }
        }
    }

    /**
     * Calculates the ideal angle to aim at the goal based on current robot position.
     * @param is_blue_alliance True if on blue alliance, false for red
     * @return Ideal heading angle in radians to face the goal
     */
    public double findIdealGoalAngle(boolean is_blue_alliance) {
        double angle = 0;
        Pose currentPose = follower.getPose();

        // Convert mm to inches for calculation
        double robotX = currentPose.getX() / 25.4;  // mm to inches
        double robotY = currentPose.getY() / 25.4;  // mm to inches

        // Find angle to point at goal
        if (is_blue_alliance) {
            // Blue alliance goal is at (-72, -72) inches
            angle = Math.atan2(-72 - robotY, -72 - robotX);
        } else {
            // Red alliance goal is at (72, -72) inches
            angle = Math.atan2(72 - robotY, -72 - robotX);
        }

        return angle;
    }

    /**
     * Aligns the robot to face the goal using AprilTag localization.
     * Returns true when aligned within tolerance.
     */
    public boolean alignToGoal() {
        // First, localize using AprilTags
        localizeViaAprilTag();

        // Calculate ideal angle to goal
        double targetAngle = findIdealGoalAngle(is_blue_alliance);
        double currentAngle = follower.getPose().getHeading();

        // Calculate heading error
        double headingError = targetAngle - currentAngle;

        // Normalize error to [-PI, PI]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        // Check if aligned within tolerance (2 degrees)
        double tolerance = Math.toRadians(2);

        if (Math.abs(headingError) < tolerance) {
            // Aligned! Stop turning
            isAligned = true;
            return true;
        } else {
            // Not aligned, command follower to turn to target angle
            Pose currentPose = follower.getPose();
            Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), targetAngle);
            follower.setPose(targetPose);

            isAligned = false;
            return false;
        }
    }

    /**
     * Translates AprilTag heading to field-centric heading.
     * @param red True if near red alliance tags, false for blue
     * @param angle_given Raw heading from AprilTag detection
     * @return Normalized heading in degrees
     */
    private double translateTCHeading(boolean red, double angle_given) {
        if (red) {
            angle_given += 90;
        } else {
            angle_given -= 90;
        } //adj for team goal heading default difference

        angle_given = angle_given % 360;
        angle_given = (angle_given + 360) % 360;

        if (angle_given > 180) {
            angle_given -= 360;
        } //normalize

        return angle_given; //heading 0 facing audience / north if X is vert.
    }

    /**
     * Builds all the paths for autonomous movement.
     * Called during init() to create the paths the robot will follow.
     */
    public void buildPaths() {
        // Preload scoring path: straight line from start to scoring position
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // First pickup cycle: path to pickup location 1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // First pickup cycle: path back to scoring from pickup 1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Second pickup cycle: curved path to pickup location 2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Second pickup cycle: straight path back to scoring from pickup 2
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Third pickup cycle: curved path to pickup location 3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Third pickup cycle: straight path back to scoring from pickup 3
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    /**
     * Autonomous state machine that controls the sequence of actions.
     * Called every loop() iteration to update the current state.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            // ============================================
            // PRELOAD CYCLE (Starting with 3 preloaded game elements)
            // ============================================

            case 0: // State 0: Drive from start position to scoring position
                follower.followPath(scorePreload, true);  // Follow path, true = stop at end
                setPathState(1);  // Move to next state immediately
                break;

            case 1: // State 1: Wait until we arrive at scoring position
                if(!follower.isBusy()) {  // Check if path following is complete
                    needsAlignment = true;  // Flag that we need to align
                    alignmentTimer.resetTimer();  // Start alignment timer
                    setPathState(2);  // Move to alignment state
                }
                break;

            case 2: // State 2: Align to goal using AprilTags
                if (alignToGoal() || alignmentTimer.getElapsedTimeSeconds() > 2.0) {
                    // Either aligned or timeout (2 seconds max for alignment)
                    needsAlignment = false;
                    Shooting(true);  // Start shooting preloads
                    setPathState(3);  // Move to shooting wait state
                }
                break;

            case 3: // State 3: Wait for all 3 preloads to be shot
                if(shotsFired >= 3) {     // Check if we've fired all 3 shots
                    Shooting(false);      // Stop shooting
                    follower.followPath(grabPickup1);  // Drive to first pickup location
                    setPathState(4);      // Move to pickup wait state
                }
                break;

            // ============================================
            // PICKUP 1 CYCLE (First set of collected game elements)
            // ============================================

            case 4: // State 4: Wait until we arrive at first pickup location
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);  // Drive back to scoring position
                    setPathState(5);      // Move to return travel state
                }
                break;

            case 5: // State 5: Wait until we return to scoring position
                if(!follower.isBusy()) {
                    needsAlignment = true;
                    alignmentTimer.resetTimer();
                    setPathState(6);  // Move to alignment state
                }
                break;

            case 6: // State 6: Align to goal
                if (alignToGoal() || alignmentTimer.getElapsedTimeSeconds() > 2.0) {
                    needsAlignment = false;
                    Shooting(true);       // Start shooting collected game elements
                    setPathState(7);      // Move to shooting wait state
                }
                break;

            case 7: // State 7: Wait for all 3 collected elements to be shot
                if(shotsFired >= 3) {
                    Shooting(false);      // Stop shooting
                    follower.followPath(grabPickup2);  // Drive to second pickup location
                    setPathState(8);      // Move to next pickup cycle
                }
                break;

            // ============================================
            // PICKUP 2 CYCLE (Second set of collected game elements)
            // ============================================

            case 8: // State 8: Wait until we arrive at second pickup location
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);  // Drive back to scoring position
                    setPathState(9);      // Move to return travel state
                }
                break;

            case 9: // State 9: Wait until we return to scoring position
                if(!follower.isBusy()) {
                    needsAlignment = true;
                    alignmentTimer.resetTimer();
                    setPathState(10);  // Move to alignment state
                }
                break;

            case 10: // State 10: Align to goal
                if (alignToGoal() || alignmentTimer.getElapsedTimeSeconds() > 2.0) {
                    needsAlignment = false;
                    Shooting(true);       // Start shooting collected game elements
                    setPathState(11);     // Move to shooting wait state
                }
                break;

            case 11: // State 11: Wait for all 3 collected elements to be shot
                if(shotsFired >= 3) {
                    Shooting(false);      // Stop shooting
                    follower.followPath(grabPickup3);  // Drive to third pickup location
                    setPathState(12);     // Move to final pickup cycle
                }
                break;

            // ============================================
            // PICKUP 3 CYCLE (Third/final set of collected game elements)
            // ============================================

            case 12: // State 12: Wait until we arrive at third pickup location
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);  // Drive back to scoring position
                    setPathState(13);     // Move to return travel state
                }
                break;

            case 13: // State 13: Wait until we return to scoring position
                if(!follower.isBusy()) {
                    needsAlignment = true;
                    alignmentTimer.resetTimer();
                    setPathState(14);  // Move to alignment state
                }
                break;

            case 14: // State 14: Align to goal
                if (alignToGoal() || alignmentTimer.getElapsedTimeSeconds() > 2.0) {
                    needsAlignment = false;
                    Shooting(true);       // Start shooting final collected elements
                    setPathState(15);     // Move to final shooting wait state
                }
                break;

            case 15: // State 15: Wait for all 3 final elements to be shot
                if(shotsFired >= 3) {
                    Shooting(false);      // Stop shooting
                    setPathState(-1);     // Move to completion state
                }
                break;

            case -1: // State -1: Autonomous routine complete
                // All actions finished, robot is idle
                break;
        }
    }

    /**
     * Initializes all hardware components (motors, servos, sensors).
     * Called during init() to set up the robot.
     */
    private void initializeHardware() {
        // Initialize servos from hardware map
        flicker = hardwareMap.get(Servo.class, "flicker");
        hood = hardwareMap.get(Servo.class, "hood");

        // Initialize motors from hardware map
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Set intake motor direction
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Enable bulk data mode for all Lynx modules (improves performance)
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Configure flywheel motor 1 for maximum RPM
        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure flywheel motor 2 for maximum RPM
        MotorConfigurationType motorConfigurationType1 = flywheel1.getMotorType().clone();
        motorConfigurationType1.setAchieveableMaxRPMFraction(1.0);
        flywheel1.setMotorType(motorConfigurationType1);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for velocity control on both flywheels
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Config.kP, Config.kI, Config.kD, Config.kF)
        );
        flywheel1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Config.kP, Config.kI, Config.kD, Config.kF)
        );

        // Set flywheel motor directions
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);

        // Initialize servo positions
        flicker.setPosition(Config.FLICKER_DEFAULT_POS);  // Default/resting position
        hood.setPosition(Config.HOOD_STARTING_POSITION);  // Starting hood position
    }

    /**
     * Controls the intake motor.
     * @param OnOff True to turn intake on, false to turn off
     */
    public void Intake(boolean OnOff){
        intakeIsOn = OnOff;
        intake.setDirection(DcMotor.Direction.REVERSE);  // Reverse direction for collecting
        intake.setPower(intakeIsOn ? Config.INTAKE_POWER : 0.0);  // Set power based on state
    }

    /**
     * Controls the shooting system (flywheels).
     * @param OnOff True to start shooting, false to stop
     */
    public void Shooting(boolean OnOff){
        shootingIsOn = OnOff;
        if (shootingIsOn) {
            // Start flywheels at configured shooting speed
            flywheel.setVelocity(Config.MEDIUMSHOTSPEED);
            flywheel1.setVelocity(Config.MEDIUMSHOTSPEED);
            shotsFired = 0;  // Reset shot counter for new shooting cycle
            if (shootTimer != null) {
                shootTimer.resetTimer();  // Reset timer for shot timing
            }
        }
        else {
            // Stop flywheels
            flywheel.setVelocity(0);
            flywheel1.setVelocity(0);
        }
    }

    /**
     * Manages the flicker servo sequence for shooting.
     * Controls timing between shots and ensures proper servo movement.
     * @param On True when shooting should be active
     */
    private void handleFlickerReload(boolean On) {
        // Check if we should start a new shot sequence
        if (On && !flickerReloading && shootTimer.getElapsedTimeSeconds() >= Config.SHOOTING_DELAY_SECONDS) {
            startFlickerSequence();  // Begin firing sequence
        }

        // Check if current flicker sequence is complete
        if (flickerReloading && flickerTimer.seconds() >= Config.SHOOTING_DELAY_SECONDS) {
            endFlickerSequence();  // Complete firing sequence
        }
    }

    /**
     * Starts the flicker servo firing sequence.
     * Moves servo to firing position and starts timing.
     */
    private void startFlickerSequence() {
        flicker.setPosition(Config.FLICKER_MAX);  // Move to firing position
        flickerTimer.reset();  // Start timing the firing duration
        flickerReloading = true;  // Mark as currently firing
    }

    /**
     * Ends the flicker servo firing sequence.
     * Returns servo to default position and updates shot count.
     */
    private void endFlickerSequence() {
        flicker.setPosition(Config.FLICKER_DEFAULT);  // Return to default position
        flickerReloading = false;  // Mark as no longer firing
        shotsFired++;  // Increment shot counter
        if (shootTimer != null) {
            shootTimer.resetTimer();  // Reset timer for next shot delay
        }
    }

    /**
     * Sets the hood servo position for shooting.
     * Adjusts the shooting angle if needed.
     */
    public void hood(){
        hood.setPosition(0.4); // TEST VALUE - adjust based on testing
    }

    /**
     * Changes the current path state and resets timing.
     * @param pState The new state to transition to
     */
    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) {
            pathTimer.resetTimer();  // Reset timer for new state
        }
    }

    /**
     * Main loop method - called repeatedly during opmode execution.
     * Contains the core autonomous logic.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // Update path follower for movement
        follower.update();

        // Update autonomous state machine
        autonomousPathUpdate();

        // Periodically localize using AprilTags when not actively aligning
        if (!needsAlignment && localizationTimer.getElapsedTimeSeconds() > 0.5) {
            localizeViaAprilTag();
            localizationTimer.resetTimer();
        }

        // Handle shooting if active and shots remaining
        if (shootingIsOn && shotsFired < 3) {
            handleFlickerReload(true);
        }

        // ============================================
        // TELEMETRY - Debug information for Driver Hub
        // ============================================
        telemetry.addData("Path State", pathState);  // Current autonomous state
        telemetry.addData("Robot X", String.format("%.2f", follower.getPose().getX()));  // X position
        telemetry.addData("Robot Y", String.format("%.2f", follower.getPose().getY()));  // Y position
        telemetry.addData("Robot Heading", String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));  // Orientation
        telemetry.addData("Aligned to Goal", isAligned);  // Alignment status
        telemetry.addData("Shots Fired", shotsFired + "/3");  // Shooting progress
        telemetry.addData("Shooting Active", shootingIsOn);  // Shooting system status
        telemetry.addData("Flicker Reloading", flickerReloading);  // Flicker servo status
        telemetry.addData("Intake Active", intakeIsOn);  // Intake system status
        telemetry.addData("AprilTags Detected", aprilTPR.getDetections().size());  // Number of tags visible

        telemetry.update();  // Send data to Driver Hub
    }

    /**
     * Initialization method - called once when INIT button is pressed.
     * Sets up all systems before autonomous starts.
     */
    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        localizationTimer = new Timer();
        alignmentTimer = new Timer();

        opmodeTimer.resetTimer();  // Start overall timer
        localizationTimer.resetTimer();  // Start localization timer

        // Set up hardware
        initializeHardware();

        // Initialize AprilTag detection
        initializeAprilTag();

        // Initialize path following system
        follower = Constants.createFollower(hardwareMap);

        // Build all autonomous paths
        buildPaths();

        // Set starting position for path following
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - Ready to start!");
        telemetry.update();
    }

    /**
     * Start method - called once when START button is pressed.
     * Begins the autonomous routine.
     */
    @Override
    public void start() {
        opmodeTimer.resetTimer();  // Reset start timer
        setPathState(0);  // Begin at first state
        hood();  // Set hood position
        Intake(true);  // Start intake
    }

    /**
     * Stop method - called when autonomous ends.
     * Clean up resources.
     */
    @Override
    public void stop() {
        // Close vision portal to free up resources
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}