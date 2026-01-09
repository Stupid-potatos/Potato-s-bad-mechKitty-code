package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assets.AprilTagReader;

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
 *
 * @author Potato
 */
@TeleOp(name="BAREBONE_TELEOP", group = "Potato's testing")
public class BAREBONESTELEOP extends OpMode {
    private final Constants robot = new Constants();


    // ============================================================================
    // PEDRO PATHING & TELEMETRY
    // ============================================================================
    private Follower follower;              // Pedro Pathing follower for advanced movement control
    private TelemetryManager telemetryM;    // Panels telemetry manager for displaying data on dashboard

    // ============================================================================
    // SENSORS
    // ============================================================================
    private VoltageSensor batteryVoltageSensor;  // Battery voltage sensor for monitoring power levels

    // ============================================================================
    // MOTORS
    // ============================================================================
    private DcMotorEx flywheel;  // Motor that spins to shoot rings
    private DcMotorEx flywheel1;
    private DcMotorEx intake;    // Motor for intake mechanism

    // ============================================================================
    // SERVOS
    // ============================================================================
    private Servo flicker;  // Servo that flicks rings into the flywheel
    private Servo hood;     // Servo for adjusting shooting angle (not currently used)
    private double servoPos = 0.5;
    private final double ServoIncrement = 0.05;
    // ============================================================================
    // SERVO POSITIONS
    // ============================================================================
    private final double flickerDefaultPos = 0.13;  // Resting position of flicker
    private final double flickerMaxPos = 1.0;      // Extended position when flicking

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
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;

    // ============================================================================
    // FLYWHEEL PIDF CONTROL
    // P = Proportional gain (responds to current error)
    // I = Integral gain (responds to accumulated error over time)
    // D = Derivative gain (responds to rate of change of error)
    // F = Feedforward gain (predicts power needed based on target velocity)
    // ============================================================================
    private final double kP = 5.0;
    private final double kI = 0.1;
    private final double kD = 0.0;
    private final double kF = 0.0;
    private final double shootingSpeed = 5000;  // Target flywheel speed
    // ============================================================================
    // April Tag and Turning to Face the April Tag initialization
    // ============================================================================
    private AprilTagReader aprilTagReader;


    /**
     * Initialization method - runs once when you press INIT on driver station
     * Sets up all hardware and prepares robot for operation
     */
    @Override
    public void  init() {

        // Initialize Panels telemetry for dashboard display
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create Pedro Pathing follower for advanced movement
        follower = Constants.createFollower(hardwareMap);

        // Get hardware from the hardware map (configured in Robot Configuration)
        flicker = hardwareMap.get(Servo.class, "flicker");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotor.Direction.FORWARD);

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

        MotorConfigurationType motorConfigurationType1 = flywheel1.getMotorType().clone();
        motorConfigurationType1.setAchieveableMaxRPMFraction(1.0);
        flywheel1.setMotorType(motorConfigurationType);
        // Enable encoder-based velocity control for precise flywheel speed
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for flywheel velocity control
        // This helps maintain consistent shooting speed
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        // Get battery voltage sensor for monitoring power
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Set flicker servo to starting position (ready to shoot)
        flicker.setPosition(flickerDefaultPos);

        // Display initialization message
        telemetryM.addLine("Robot initialized - Ready to go!");
        telemetryM.update();

        aprilTagReader = new AprilTagReader(hardwareMap);  // Start AprilTag detection

        Pose startingPose = new Pose(0,0,0);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
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
        double drive = gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;   // Left/right strafing
        double turn = gamepad1.right_stick_x;    // Rotation
        if (follower != null) {
            // Send drive commands to the follower and update robot position
            follower.setTeleOpDrive(drive, strafe, turn);
            follower.update();
        }
        else {follower.setStartingPose(new Pose(0, 0, 0));}
        // === BUTTON DETECTION (Edge Detection for Toggles) ===
        // Edge detection: only trigger when button goes from not pressed to pressed
        // This prevents holding a button from toggling the mechanism repeatedly
        boolean shootButtonPressed = gamepad1.yWasPressed();
        boolean intakeButtonPressed = gamepad1.aWasPressed();
        //boolean outtakeButtonPressed = gamepad1.square && !prevOuttakeButton;
        boolean aprilTagButtonPressed = gamepad1.circle && !prevAprilTagButton;
        boolean reloadButtonPressed = gamepad1.left_bumper && !prevReloadButton;
        boolean hoodUP = gamepad1.dpad_right;
        boolean hoodDown = gamepad1.dpad_left;


        // Update previous button states for next loop iteration
        //prevShootButton = gamepad1.a;
        //prevIntakeButton = gamepad1.xWasPressed();
        prevOuttakeButton = gamepad1.square;
        prevAprilTagButton = gamepad1.circle;
        prevReloadButton = gamepad1.left_bumper;

        // === INTAKE TOGGLE ===
        // Press X button to turn intake on/off
        if (intakeButtonPressed) {
            intakeIsOn = !intakeIsOn;
            outtakeIsOn = false;
            intake.setPower(intakeIsOn ? -1 : 0);
            }

        // === SHOOTING TOGGLE ===
        // Press A button to turn shooting on/off
        if (shootButtonPressed) {
            shootingIsOn = !shootingIsOn;  // Flip the state
            flywheel.setVelocity(shootingIsOn ? shootingSpeed : 0);
            flywheel1.setVelocity(shootingIsOn ? shootingSpeed : 0);
        }

        // === OUTTAKE TOGGLE ===
        // Press Square button to turn outtake on/off
        //if (outtakeButtonPressed) {
        //    outtakeIsOn = !outtakeIsOn;
        //    intakeIsOn = false;
        //    intake.setPower(outtakeIsOn ? -0.5 : 0);
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

        if (hoodUP) {
            servoPos += ServoIncrement;
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            hood.setPosition(servoPos);
        }
        prevDpadRight = gamepad1.dpad_right;
        if (hoodDown) {
            servoPos -= ServoIncrement;
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            hood.setPosition(servoPos);
        }



        // === TELEMETRY - Position data ===
        telemetryM.addLine("=== Position ===");
        telemetryM.addLine("X: " + String.format("%.2f", follower.getPose().getX()));
        telemetryM.addLine("Y: " + String.format("%.2f", follower.getPose().getY()));
        telemetryM.addLine("BATTERY: " + String.format("%.2fV", batteryVoltageSensor.getVoltage()));

        // === TELEMETRY - Mechanism status ===
        telemetryM.addLine("\n=== Mechanisms ===");
        telemetryM.addLine("Shooting: " + (shootingIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Intake: " + (intakeIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Outtake: " + (outtakeIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Flicker: " + (flickerReloading ? "RELOADING" : "READY"));
        telemetryM.addLine("Hood: " + (servoPos));

        telemetry.addLine("Shooting: " + ((flywheel.getVelocity(AngleUnit.DEGREES)/28)*60));
        telemetry.addLine("Hood: " + (servoPos)); // NEED TUNING
        // Update telemetry display
        telemetryM.update();

    }

    /**
     * Stop method - runs once when OpMode is stopped
     * Safely shuts down all mechanisms
     */
    @Override
    public void stop() {
        // Stop all mechanisms when OpMode ends
        if (flywheel != null) {
            flywheel.setPower(0);
            flywheel1.setPower(0);
        }
        if (intake != null) intake.setPower(0);
        if (flicker != null) flicker.setPosition(flickerDefaultPos);
        if (follower != null) {
            follower.breakFollowing();
            follower.setTeleOpDrive(0, 0, 0);
        }
        if (aprilTagReader != null) aprilTagReader.close();  // Clean up camera
    }
}