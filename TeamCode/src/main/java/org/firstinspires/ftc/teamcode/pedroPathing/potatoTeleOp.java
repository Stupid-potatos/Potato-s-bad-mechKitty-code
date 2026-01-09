package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.bylazar.field.FieldManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.field.PanelsField;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assets.AprilTagReader;
import org.firstinspires.ftc.teamcode.Potato_Assets.TurnToFaceTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * TeleOp mode with shooting, intake, and outtake mechanisms
 * Uses toggle buttons for mechanism control
 *
 * Controls:
 * - Left stick Y: Forward/Backward
 * - Left stick X: Strafe left/right
 * - Right stick X: Rotation
 * - A: Toggle shooting
 * - X: Toggle intake
 * - Square: Toggle outtake
 * - Circle: Scan AprilTag
 * - Left Bumper: Reload/flick
 * - Right Bumper: Turn to face AprilTag
 *
 * @author Potato
 */
@TeleOp(name="PotatoTeleOp", group = "Potato's testing")
public class potatoTeleOp extends OpMode {
    // ============================================================================
    // PEDRO PATHING & TELEMETRY
    // ============================================================================
    private Follower follower;              // Pedro Pathing follower for advanced movement control
    private TelemetryManager telemetryM;    // Panels telemetry manager for displaying data on dashboard
    private FieldManager field;             // Field manager for visualizing robot position on Panels

    // ============================================================================
    // SENSORS
    // ============================================================================
    private VoltageSensor batteryVoltageSensor;  // Battery voltage sensor for monitoring power levels

    // ============================================================================
    // MOTORS
    // ============================================================================
    private DcMotorEx flywheel;  // Motor that spins to shoot rings
    private DcMotorEx Intake;    // Motor for intake mechanism

    // ============================================================================
    // SERVOS
    // ============================================================================
    private Servo flicker;  // Servo that flicks rings into the flywheel
    private Servo hood;     // Servo for adjusting shooting angle (not currently used)
    private double servoPos = 0.5;

    // ============================================================================
    // SERVO POSITIONS
    // ============================================================================
    private final double flickerDefaultPos = 0.13;  // Resting position of flicker
    private final double flickerMaxPos = 0.43;      // Extended position when flicking

    // ============================================================================
    // MECHANISM STATE FLAGS
    // ============================================================================
    private boolean shootingIsOn = false;
    private boolean intakeIsOn = false;
    private boolean outtakeIsOn = false;

    // ============================================================================
    // FLICKER TIMING & RELOAD
    // ============================================================================
    private ElapsedTime flickerTimer = new ElapsedTime();  // Timer to track flicker movement
    private boolean flickerReloading = false;               // Flag to track if flicker is currently moving
    private final double shootingDelaySeconds = 0.3;        // Time (in seconds) flicker stays extended

    // ============================================================================
    // BUTTON EDGE DETECTION
    // Previous button states prevent holding a button from toggling repeatedly
    // Edge detection = only trigger on the moment button is pressed, not while held
    // ============================================================================
    private boolean prevShootButton = false;
    private boolean prevIntakeButton = false;
    private boolean prevOuttakeButton = false;
    private boolean prevAprilTagButton = false;
    private boolean prevReloadButton = false;

    // ============================================================================
    // FLYWHEEL PIDF CONTROL
    // P = Proportional gain (responds to current error)
    // I = Integral gain (responds to accumulated error over time)
    // D = Derivative gain (responds to rate of change of error)
    // F = Feedforward gain (predicts power needed based on target velocity)
    // ============================================================================
    private final double kP = 5.0;
    private final double kI = 0.1;
    private final double kD = 0.2;
    private final double kF = 12.0;
    private final double shootingSpeed = 1.0;  // Target flywheel speed
    // ============================================================================
    // April Tag and Turning to Face the April Tag initialization
    // ============================================================================
    private AprilTagReader aprilTagReader;
    private TurnToFaceTag turnToFaceTag;


    /**
     * Initialization method - runs once when you press INIT on driver station
     * Sets up all hardware and prepares robot for operation
     */
    @Override
    public void init() {
        // Initialize Panels telemetry for dashboard display
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create Pedro Pathing follower for advanced movement
        follower = Constants.createFollower(hardwareMap);

        // Get hardware from the hardware map (configured in Robot Configuration)
        flicker = hardwareMap.get(Servo.class, "flicker");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Initialize the Panels field visualization
        field = PanelsField.INSTANCE.getField();


        // Enable bulk reading for better performance
        // This reads all sensor data at once instead of individually
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Remove the default 85% speed limit on the flywheel motor
        // This allows the motor to run at full power
        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);

        // Enable encoder-based velocity control for precise flywheel speed
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for flywheel velocity control
        // This helps maintain consistent shooting speed
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        // Get battery voltage sensor for monitoring power
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Set flicker servo to starting position (ready to shoot)
        flicker.setPosition(flickerDefaultPos);

        // Display initialization message
        telemetryM.addLine("Robot initialized - Ready to go!");
        telemetryM.update();

        aprilTagReader = new AprilTagReader(hardwareMap);  // Start AprilTag detection
        //turnToFaceTag = new TurnToFaceTag(follower, telemetry); // Initialize turn utility

        follower.setPose(new Pose(0, 0, 0)); // x, y, heading (radians)

    }

    /**
     * Loop method - runs repeatedly while OpMode is active
     * Handles all robot control and telemetry updates
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // === DRIVING CONTROL ===
        // Read gamepad joystick values for robot movement
        double drive = -gamepad1.left_stick_y;  // Forward/backward (inverted because Y is reversed)
        double strafe = gamepad1.left_stick_x;   // Left/right strafing
        double turn = gamepad1.right_stick_x;    // Rotation

        Pose pose = follower.getPose();
        if (pose != null) {
            follower.setTeleOpDrive(drive, strafe, turn);
            follower.update();
        }

        // === BUTTON DETECTION (Edge Detection for Toggles) ===
        // Edge detection: only trigger when button goes from not pressed to pressed
        // This prevents holding a button from toggling the mechanism repeatedly
        boolean shootButtonPressed = gamepad1.a && !prevShootButton;
        boolean intakeButtonPressed = gamepad1.x && !prevIntakeButton;
        boolean outtakeButtonPressed = gamepad1.square && !prevOuttakeButton;
        boolean aprilTagButtonPressed = gamepad1.circle && !prevAprilTagButton;
        boolean reloadButtonPressed = gamepad1.left_bumper && !prevReloadButton;

        // Update previous button states for next loop iteration
        prevShootButton = gamepad1.a;
        prevIntakeButton = gamepad1.x;
        prevOuttakeButton = gamepad1.square;
        prevAprilTagButton = gamepad1.circle;
        prevReloadButton = gamepad1.left_bumper;

        // === INTAKE TOGGLE ===
        // Press X button to turn intake on/off
        if (intakeButtonPressed) {
            intakeIsOn = !intakeIsOn;  // Flip the state

            Intake.setPower(intakeIsOn ? 0.3 : 0.0);  // Turn motor on (0.3) or off (0.0)
        }

        // === SHOOTING TOGGLE ===
        // Press A button to turn shooting on/off
        if (shootButtonPressed) {
            
            shootingIsOn = !shootingIsOn;  // Flip the state
            flywheel.setPower(shootingIsOn ? shootingSpeed : 0.0);
        }

        // === OUTTAKE TOGGLE ===
        // Press Square button to turn outtake on/off
        //if (outtakeButtonPressed) {
        //    outtakeIsOn = !outtakeIsOn;  // Flip the state
        //    if (outtakeIsOn) {
        //        intakeIsOn = false;  // Turn off intake when outtaking
        //        Intake.setPower(-0.3);
        //}

        // === RELOAD/FLICKER ===
        // Press left bumper to flick a ring
        if (reloadButtonPressed && !flickerReloading) {
            // Start the flicker sequence
            flicker.setPosition(flickerMaxPos);  // Move flicker forward
            flickerTimer.reset();                 // Start timing the movement
            flickerReloading = true;              // Mark that we're in reload sequence
        }

        // Check if flicker needs to return to default position
        // After the delay time has passed, retract the flicker
        if (flickerReloading && flickerTimer.seconds() >= shootingDelaySeconds) {
            flicker.setPosition(flickerDefaultPos);  // Move flicker back to starting position
            flickerReloading = false;                 // Mark reload sequence as complete
        }

        // === APRILTAG SCAN ===
        // Press Circle button to check for AprilTag detection
        if (aprilTagButtonPressed) {
            // Check if AprilTag reader detected a motif (pattern)
            if (aprilTagReader.getMotif() != null) {
                telemetryM.addLine("Motif: " + aprilTagReader.getMotif());
            } else {
                telemetryM.addLine("No Tag Found");
            }
        }

        //if (gamepad1.right_bumper) {
        //    List<AprilTagDetection> detections = aprilTagReader.getFilteredDetections();
        //    turnToFaceTag.executeTurn(detections);
        //}

        if (pose == null) return;

        if (gamepad1.dpadRightWasPressed()) {
            servoPos += 0.01;
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            hood.setPosition(servoPos);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            servoPos -= 0.01;
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            hood.setPosition(servoPos);
        }


        // === TELEMETRY - Position data ===
        telemetryM.addLine("=== Position ===");
        telemetryM.addLine("X: " + String.format("%.2f", pose.getX()));
        telemetryM.addLine("Y: " + String.format("%.2f", pose.getY()));
        //telemetryM.addLine("Heading: " + String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.addLine("BATTERY: " + String.format("%.2fV", batteryVoltageSensor.getVoltage()));

        // === TELEMETRY - Mechanism status ===
        telemetryM.addLine("\n=== Mechanisms ===");
        telemetryM.addLine("Shooting: " + (shootingIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Intake: " + (intakeIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Outtake: " + (outtakeIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Flicker: " + (flickerReloading ? "RELOADING" : "READY"));
        telemetryM.addLine("Hood: " + (servoPos));

        // Update telemetry display
        telemetryM.update();

        // === PANELS FIELD VISUALIZATION ===
        // Get current robot pose (position and heading)

        // Update the cursor position on the Panels field view
        // This shows where the robot is on the virtual field
        field.moveCursor(pose.getX(), pose.getY());
        field.update();
    }

    /**
     * Stop method - runs once when OpMode is stopped
     * Safely shuts down all mechanisms
     */
    @Override
    public void stop() {
        try {
            // Stop all mechanisms when OpMode ends
            if (flywheel != null) {
                flywheel.setPower(0);
            }
            if (Intake != null) Intake.setPower(0);

            if (follower != null) {
                follower.breakFollowing();
            }

        }
        catch (Exception e) {

        }
    }
}