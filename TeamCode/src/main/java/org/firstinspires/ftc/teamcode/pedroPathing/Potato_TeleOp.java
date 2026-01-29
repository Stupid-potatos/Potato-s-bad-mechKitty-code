package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Potato_Assets.Config;
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
 * - Y: Toggle shooting
 * - B: Toggle intake
 * - Circle: Scan AprilTag
 * - Left Bumper: Reload/flick
 * - Left Trigger: turn to goal
 * - D-pad up: Increase hood position
 * - D-pad down: Decrease hood position
 *
 * @author Potato
 * @author ricxuuuu
 * TODO:
 *      can get rid of motif finder, that just needs to work during testing to get correct motif for auton
 *
 */
@TeleOp(name = "Potato_TeleOp", group = "Potato's testing")
public class Potato_TeleOp extends OpMode {

    // ========================================
    // CONSTANTS
    // ========================================

    /*
    Change before game to determine which alliance you are on!!!!
     */
    boolean is_blue_alliance = true;
    boolean next_to_goal = false;


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
    private ElapsedTime hoodTimer = new ElapsedTime();
    Pose StartingPose;

    // ========================================
    // INITIALIZATION
    // ========================================

    @Override
    public void init() {
        // WILL GET REMOVED DURING COMP
        Pose startingPose = is_blue_alliance
                ? (next_to_goal ? Config.BLUE_NEAR_GOAL_START : Config.BLUE_FAR_START)
                : (next_to_goal ? Config.RED_NEAR_GOAL_START : Config.RED_FAR_START);

        follower.setStartingPose(startingPose);
        initializeTelemetry();
        initializeHardware();
        configureBulkReading();
        configureFlywheelMotor();
        configurePIDFCoefficients();
        initializeSensors();
        setInitialPositions();

        telemetry.addLine("Robot initialized - Ready to go!");
        telemetry.update();
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
                new PIDFCoefficients(Config.kP, Config.kI, Config.kD, Config.kF)
        );
        flywheel1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Config.kP, Config.kI, Config.kD, Config.kF)
        );
    }

    private void initializeSensors() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        aprilTagReader = new AprilTagReader(hardwareMap);
    }

    private void setInitialPositions() {
        flicker.setPosition(Config.FLICKER_DEFAULT_POS);
        hood.setPosition(Config.HOOD_STARTING_POSITION);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    // ========================================
    // MAIN LOOP
    // ========================================

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        updateDriving();
        VariableFlyWheelSpeeds(findHypotenuseFromGoal());
        turnToGoal();
        handleMechanismToggles();
        updateFlywheelSpeed();
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
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(drive, strafe, turn);
        follower.update();
    }

    // ========================================
    // MECHANISM CONTROL
    // ========================================

    private void handleMechanismToggles() {
        handleShootingToggle();
        handleIntakeToggle();
    }

    private void handleShootingToggle() {
        if (gamepad1.bWasPressed()) {
            shootingIsOn = !shootingIsOn;
            updateFlywheelSpeed();
        }
    }
    private void updateFlywheelSpeed() {
        if (shootingIsOn) {
            flywheel.setVelocity(Config.SHOOTINGSPEEDCURRENT);
            flywheel1.setVelocity(Config.SHOOTINGSPEEDCURRENT);
        } else {
            flywheel.setVelocity(0);
            flywheel1.setVelocity(0);
        }
    }
    private void handleIntakeToggle() {
        if (gamepad1.aWasPressed()) {
            intakeIsOn = !intakeIsOn;
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(intakeIsOn ? Config.INTAKE_POWER : 0.0);
        }
    }
    // ========================================
    // FLICKER CONTROL
    // ========================================
    private void handleFlickerReload() {
        if (gamepad1.leftBumperWasPressed() && !flickerReloading) {
            startFlickerSequence();
        }

        if (flickerReloading && flickerTimer.seconds() >= Config.SHOOTING_DELAY_SECONDS) {
            endFlickerSequence();
        }
    }

    private void startFlickerSequence() {
        flicker.setPosition(Config.FLICKER_MAX);
        flickerTimer.reset();
        flickerReloading = true;
    }

    private void endFlickerSequence() {
        flicker.setPosition(Config.FLICKER_DEFAULT);
        flickerReloading = false;
    }

    // ========================================
    // APRILTAG SCANNING TEMP
    // ========================================
    private void handleAprilTagScan() {
        if (gamepad1.circleWasPressed()) {
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
        if (hoodTimer.seconds() >= Config.HOOD_ADJUST_DELAY) {
            if (gamepad1.dpad_up) {
                adjustHoodPosition(Config.SERVO_INCREMENT);
            }
            if (gamepad1.dpad_down) {
                adjustHoodPosition(-Config.SERVO_INCREMENT);
            }
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

    private double[] getGoalPosition() {
        return is_blue_alliance
                ? new double[]{Config.BLUE_GOAL_X, Config.BLUE_GOAL_Y}
                : new double[]{Config.RED_GOAL_X, Config.RED_GOAL_Y};
    }
    private void turnToGoal(){
        if (gamepad1.left_trigger > 0.13 && !follower.isBusy()) {
            follower.turnTo(findIdealGoalAngle());
            double hoodAngle = CalculateHoodAngle(findHypotenuseFromGoal()) / Config.MAX_HOOD_ANGLE; // COULD remove max_hood_angle.
            hood.setPosition(hoodAngle);
            telemetry.addLine("Distance: "+ findHypotenuseFromGoal());
            telemetry.addLine("Theoretical hood angle " + CalculateHoodAngle(findHypotenuseFromGoal()));
            telemetry.update();
        }
        follower.update();
    }
    /* this ver will most likely work
    private void turnToGoal(){
        if (gamepad1.left_trigger > 0.13) {
            double targetHeading = findIdealGoalAngle();

            // Calculate turn amount needed
            double currentHeading = follower.getPose().getHeading();
            double headingError = targetHeading - currentHeading;

            // Normalize to -PI to PI
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Apply proportional turn control
            double turnPower = headingError * 0.5; // Adjust multiplier as needed
            turnPower = Math.max(-1, Math.min(1, turnPower)); // Clamp to [-1, 1]

            follower.setTeleOpDrive(0, 0, turnPower);

            // Update hood
            double hoodAngle = CalculateHoodAngle(findHypotenuseFromGoal()) / Config.MAX_HOOD_ANGLE;
            hood.setPosition(hoodAngle);

            telemetry.addLine("Distance: "+ findHypotenuseFromGoal());
            telemetry.addLine("Theoretical hood angle " + CalculateHoodAngle(findHypotenuseFromGoal()));
        }
    }
     */
    public double findIdealGoalAngle() {
        double[] goal = getGoalPosition();
        return Math.atan2(goal[1] - follower.getPose().getY(),
                goal[0] - follower.getPose().getX());
    }

    // For when we get formula for turning hood position to goal
    public double findHypotenuseFromGoal() {
        double[] goal = getGoalPosition();
        return Math.hypot(goal[0] - follower.getPose().getX(),
                goal[1] - follower.getPose().getY());
    }

    public double CalculateHoodAngle(double distance){
        double [][] COEFFICIENTS = {
                // {m,b}
                {1,0.5},
                {1.3,2},
                {2.3,6},
        };
        int selector = VariableFlyWheelSpeeds(distance);
        double m = COEFFICIENTS[selector][0];
        double b = COEFFICIENTS[selector][1];
        return m * distance + b;
        }


    public int VariableFlyWheelSpeeds(double distance){
        int speedZone;

        if (distance <= 10.0) {
            speedZone = 0;
        } else if (distance <= 30.0) {
            speedZone = 1;
        } else {
            speedZone = 2; // Everything 5+ uses far shot
        }

        // Set flywheel speed based on zone
        switch (speedZone) {
            case 0:
                Config.SHOOTINGSPEEDCURRENT = Config.CLOSESHOTSPEED;
                break;
            case 1:
                Config.SHOOTINGSPEEDCURRENT = Config.MEDIUMSHOTSPEED;
                break;
            case 2:
                Config.SHOOTINGSPEEDCURRENT = Config.FARSHOTSPEED;
                break;
        }

        return speedZone;
    }

    // ========================================
    // TELEMETRY
    // ========================================

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        displayPositionData();
        displayMechanismStatus();
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    private void displayPositionData() {
        telemetry.addLine("=== Position ===");
        telemetry.addLine("X: " + String.format("%.2f", follower.getPose().getX()));
        telemetry.addLine("Y: " + String.format("%.2f", follower.getPose().getY()));
        telemetry.addLine("BATTERY: " + String.format("%.2fV", batteryVoltageSensor.getVoltage()));
    }

    @SuppressLint("DefaultLocale")
    private void displayMechanismStatus() {
        double distance = findHypotenuseFromGoal();
        int speedZone = getSpeedZone(distance); // New helper method
        String zoneName = getZoneName(speedZone);
        telemetry.addLine("\n=== Mechanisms ===");
        telemetry.addLine("Shooting: " + (shootingIsOn ? "ON" : "OFF"));
        telemetry.addLine("FlyWheel Speed: " + Config.SHOOTINGSPEEDCURRENT);
        telemetry.addLine("Intake: " + (intakeIsOn ? "ON" : "OFF"));
        telemetry.addLine("Outtake: " + (outtakeIsOn ? "ON" : "OFF"));
        telemetry.addLine("Flicker: " + (flickerReloading ? "RELOADING" : "READY"));
        telemetry.addLine("Hood: " + String.format("%.3f", hood.getPosition()));
        telemetry.addLine("Zone: " + zoneName + " (" + speedZone + ")");
    }
    private int getSpeedZone(double distance) {
        if (distance <= 2.0) return 0;
        if (distance <= 5.0) return 1;
        return 2;
    }

    private String getZoneName(int zone) {
        switch (zone) {
            case 0: return "CLOSE";
            case 1: return "MEDIUM";
            case 2: return "FAR";
            default: return "UNKNOWN";
        }
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
        if (flicker != null) flicker.setPosition(Config.FLICKER_DEFAULT);
        if (hood != null) hood.setPosition(Config.HOOD_STARTING_POSITION);

        if (follower != null) {
            follower.breakFollowing();
            follower.setTeleOpDrive(0, 0, 0);
        }
    }
    private void cleanupResources() {
        if (aprilTagReader != null) aprilTagReader.close();
    }
}