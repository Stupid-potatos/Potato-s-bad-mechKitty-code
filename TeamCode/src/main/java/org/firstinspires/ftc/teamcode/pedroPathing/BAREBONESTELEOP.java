package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assets.AprilTagReader;

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
 * - D-pad Right: Increase hood position
 * - D-pad Left: Decrease hood position
 *
 * @author Potato
 */
@TeleOp(name = "BAREBONE_TELEOP", group = "Potato's testing")
public class BAREBONESTELEOP extends OpMode {

    // ========================================
    // CONSTANTS
    // ========================================

    // Servo positions
    private static final double FLICKER_DEFAULT_POS = 0.13;
    private static final double FLICKER_MAX_POS = 0.45;
    private static final double SERVO_INCREMENT = 0.02;
    private static final double HOOD_STARTING_POSITION = 0.00; // TEMP VALUE

    // Motor settings
    private static final double INTAKE_POWER = 0.3;
    private static final double SHOOTING_SPEED = 3000;
    private static final double SHOOTING_DELAY_SECONDS = 0.3;

    // PIDF coefficients TEMP
    private static final double kP = 5.0;
    private static final double kI = 0.1;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    // ========================================
    // HARDWARE COMPONENTS
    // ========================================

    // Pedro Pathing & Telemetry
    private Follower follower;
    private TelemetryManager telemetryM;

    // Sensors
    private VoltageSensor batteryVoltageSensor;
    private AprilTagReader aprilTagReader;

    // Motors
    private DcMotorEx flywheel;
    private DcMotorEx flywheel1;
    private DcMotorEx intake;

    // Servos
    private Servo flicker;
    private Servo hood;

    // ========================================
    // STATE VARIABLES
    // ========================================

    // Mechanism states
    private boolean shootingIsOn = false;
    private boolean intakeIsOn = false;
    private boolean outtakeIsOn = false;

    // Flicker control
    private ElapsedTime flickerTimer = new ElapsedTime();
    private boolean flickerReloading = false;

    // Hood position
    private double servoPos = 0.5;

    // Button edge detection
    private boolean prevShootButton = false;
    private boolean prevIntakeButton = false;
    private boolean prevOuttakeButton = false;
    private boolean prevAprilTagButton = false;
    private boolean prevReloadButton = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;

    // ========================================
    // INITIALIZATION
    // ========================================

    @Override
    public void init() {
        initializeTelemetry();
        initializeHardware();
        configureBulkReading();
        configureFlywheelMotor();
        configurePIDFCoefficients();
        initializeSensors();
        setInitialPositions();

        telemetryM.addLine("Robot initialized - Ready to go!");
        telemetryM.update();
    }

    private void initializeTelemetry() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
    }

    private void initializeHardware() {
        flicker = hardwareMap.get(Servo.class, "flicker");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    private void configureBulkReading() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    private void configureFlywheelMotor() {
        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType1 = flywheel1.getMotorType().clone();
        motorConfigurationType1.setAchieveableMaxRPMFraction(1.0);
        flywheel1.setMotorType(motorConfigurationType1);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void configurePIDFCoefficients() {
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
        flywheel1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
    }

    private void initializeSensors() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        aprilTagReader = new AprilTagReader(hardwareMap);
    }

    private void setInitialPositions() {
        flicker.setPosition(FLICKER_DEFAULT_POS);
        hood.setPosition(HOOD_STARTING_POSITION);
    }

    // ========================================
    // MAIN LOOP
    // ========================================

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        updateDriving();
        updateButtonStates();
        handleMechanismToggles();
        handleFlickerReload();
        handleAprilTagScan();
        handleHoodAdjustment();
        updateTelemetry();
    }

    // ========================================
    // DRIVING CONTROL
    // ========================================

    private void updateDriving() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        follower.setTeleOpDrive(drive, strafe, turn);
        follower.update();
    }

    // ========================================
    // BUTTON MANAGEMENT
    // ========================================

    private void updateButtonStates() {
        prevShootButton = gamepad1.a;
        prevIntakeButton = gamepad1.x;
        prevOuttakeButton = gamepad1.square;
        prevAprilTagButton = gamepad1.circle;
        prevReloadButton = gamepad1.left_bumper;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadLeft = gamepad1.dpad_left;
    }

    private boolean isButtonPressed(boolean current, boolean previous) {
        return current && !previous;
    }

    // ========================================
    // MECHANISM CONTROL
    // ========================================

    private void handleMechanismToggles() {
        handleShootingToggle();
        handleIntakeToggle();
        handleOuttakeToggle();
    }

    private void handleShootingToggle() {
        if (isButtonPressed(gamepad1.a, prevShootButton)) {
            shootingIsOn = !shootingIsOn;
            flywheel.setVelocity(shootingIsOn ? SHOOTING_SPEED : 0);

        }
    }

    private void handleIntakeToggle() {
        if (isButtonPressed(gamepad1.x, prevIntakeButton)) {
            outtakeIsOn = false;
            intakeIsOn = !intakeIsOn;
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setPower(intakeIsOn ? INTAKE_POWER : 0.0);
        }
    }

    private void handleOuttakeToggle() {
        if (isButtonPressed(gamepad1.square, prevOuttakeButton)) {
            outtakeIsOn = !outtakeIsOn;
            intakeIsOn = false;
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(outtakeIsOn ? INTAKE_POWER : 0.0);
        }
    }

    // ========================================
    // FLICKER CONTROL
    // ========================================

    private void handleFlickerReload() {
        if (isButtonPressed(gamepad1.left_bumper, prevReloadButton) && !flickerReloading) {
            startFlickerSequence();
        }

        if (flickerReloading && flickerTimer.seconds() >= SHOOTING_DELAY_SECONDS) {
            endFlickerSequence();
        }
    }

    private void startFlickerSequence() {
        flicker.setPosition(FLICKER_MAX_POS);
        flickerTimer.reset();
        flickerReloading = true;
    }

    private void endFlickerSequence() {
        flicker.setPosition(FLICKER_DEFAULT_POS);
        flickerReloading = false;
    }

    // ========================================
    // APRILTAG SCANNING TEMP
    // ========================================

    private void handleAprilTagScan() {
        if (isButtonPressed(gamepad1.circle, prevAprilTagButton)) {
            if (aprilTagReader.getMotif() != null) {
                telemetryM.addLine("Motif: " + aprilTagReader.getMotif());
            } else {
                telemetryM.addLine("No Tag Found");
            }
        }
    }

    // ========================================
    // HOOD ADJUSTMENT
    // ========================================

    private void handleHoodAdjustment() {
        if (isButtonPressed(gamepad1.dpad_right, prevDpadRight)) {
            adjustHoodPosition(SERVO_INCREMENT);
        }

        if (isButtonPressed(gamepad1.dpad_left, prevDpadLeft)) {
            adjustHoodPosition(-SERVO_INCREMENT);
        }
    }

    private void adjustHoodPosition(double change) {
        servoPos += change;
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        hood.setPosition(servoPos);
    }

    // ========================================
    // TURN TO GOAL
    // ========================================
    private void turnToGoal(){
        
    }
    // ========================================
    // TELEMETRY
    // ========================================

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        displayPositionData();
        displayMechanismStatus();
        telemetryM.update();
    }

    private void displayPositionData() {
        telemetryM.addLine("=== Position ===");
        telemetryM.addLine("X: " + String.format("%.2f", follower.getPose().getX()));
        telemetryM.addLine("Y: " + String.format("%.2f", follower.getPose().getY()));
        telemetryM.addLine("BATTERY: " + String.format("%.2fV", batteryVoltageSensor.getVoltage()));
    }

    private void displayMechanismStatus() {
        telemetryM.addLine("\n=== Mechanisms ===");
        telemetryM.addLine("Shooting: " + (shootingIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Intake: " + (intakeIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Outtake: " + (outtakeIsOn ? "ON" : "OFF"));
        telemetryM.addLine("Flicker: " + (flickerReloading ? "RELOADING" : "READY"));
        telemetryM.addLine("Hood: " + servoPos);
    }

    // ========================================
    // CLEANUP
    // ========================================

    @Override
    public void stop() {
        stopAllMechanisms();
        cleanupResources();
    }

    private void stopAllMechanisms() {
        if (flywheel != null) flywheel.setPower(0);
        if (flywheel1 != null) flywheel1.setPower(0);
        if (intake != null) intake.setPower(0);
        if (flicker != null) flicker.setPosition(FLICKER_DEFAULT_POS);
        if (hood != null) hood.setPosition(HOOD_STARTING_POSITION);

        if (follower != null) {
            follower.breakFollowing();
            follower.setTeleOpDrive(0, 0, 0);
        }
    }

    private void cleanupResources() {
        if (aprilTagReader != null) aprilTagReader.close();
    }
}