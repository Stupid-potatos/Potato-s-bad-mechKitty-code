package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Potato_Assets.Config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BLUE 1-Cycle Test Auto", group = "Potato's testing")
public class test_auto_1cycle extends OpMode {
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
    private DcMotorEx flywheel;
    private DcMotorEx flywheel1;
    private DcMotorEx intake;
    boolean intakeIsOn = false;
    private ElapsedTime flickerTimer = new ElapsedTime();
    private boolean flickerReloading = false;

    // Shooting system state
    boolean shootingIsOn;
    int shotsFired = 0;
    boolean isAligned = false;
    boolean needsAlignment = false;

    // ============================================
    // SERVO DECLARATIONS
    // ============================================
    private Servo flicker;
    private Servo hood;

    // ============================================
    // PATH FOLLOWING SYSTEM
    // ============================================
    private Follower follower;
    private Timer pathTimer;
    private Timer shootTimer;
    private Timer opmodeTimer;
    private Timer localizationTimer;
    private Timer alignmentTimer;

    private int pathState;

    // ============================================
    // POSITION DEFINITIONS (1 CYCLE ONLY)
    // ============================================
    private final Pose startPose = Config.BLUE_NEAR_GOAL_START;
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(14.415827338129493, 83.91223021582736, Math.toRadians(180));

    // ============================================
    // PATH DEFINITIONS (1 CYCLE ONLY)
    // ============================================
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;

    private void initializeAprilTag() {
        aprilTPR = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTPR)
                .build();
    }

    public void localizeViaAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTPR.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 && !is_blue_alliance) {
                hallucination = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                if (hallucination < 90 && hallucination > 0) {
                    double x = detection.robotPose.getPosition().x;
                    double y = detection.robotPose.getPosition().y;
                    double heading = Math.toRadians(translateTCHeading(true, hallucination));
                    follower.setPose(new Pose(x, y, heading));
                    telemetry.addLine("Localized via AprilTag 24!");
                }
            }

            if (detection.id == 20 && is_blue_alliance) {
                hallucination = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                if (hallucination < 0 && hallucination > -90) {
                    double x = detection.robotPose.getPosition().x;
                    double y = detection.robotPose.getPosition().y;
                    double heading = Math.toRadians(translateTCHeading(false, hallucination));
                    follower.setPose(new Pose(x, y, heading));
                    telemetry.addLine("Localized via AprilTag 20!");
                }
            }
        }
    }

    public double findIdealGoalAngle(boolean is_blue_alliance) {
        double angle = 0;
        Pose currentPose = follower.getPose();

        double robotX = currentPose.getX() / 25.4;  // mm to inches
        double robotY = currentPose.getY() / 25.4;  // mm to inches

        if (is_blue_alliance) {
            angle = Math.atan2(-72 - robotY, -72 - robotX);
        } else {
            angle = Math.atan2(72 - robotY, -72 - robotX);
        }

        return angle;
    }

    public boolean alignToGoal() {
        localizeViaAprilTag();

        double targetAngle = findIdealGoalAngle(is_blue_alliance);
        double currentAngle = follower.getPose().getHeading();
        double headingError = targetAngle - currentAngle;

        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        double tolerance = Math.toRadians(2);

        if (Math.abs(headingError) < tolerance) {
            isAligned = true;
            return true;
        } else {
            Pose currentPose = follower.getPose();
            Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), targetAngle);
            follower.setPose(targetPose);
            isAligned = false;
            return false;
        }
    }

    private double translateTCHeading(boolean red, double angle_given) {
        if (red) {
            angle_given += 90;
        } else {
            angle_given -= 90;
        }
        angle_given = angle_given % 360;
        angle_given = (angle_given + 360) % 360;
        if (angle_given > 180) {
            angle_given -= 360;
        }
        return angle_given;
    }

    public void buildPaths() {
        // Preload scoring path
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path to pickup location 1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Path back to scoring from pickup 1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    /**
     * SIMPLIFIED 1-CYCLE AUTONOMOUS STATE MACHINE
     *
     * Flow: Start → Score Preload (3 shots) → Pickup 1 → Return → Score (3 shots) → Done
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            // ============================================
            // PRELOAD CYCLE
            // ============================================
            case 0: // Drive to score position
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1: // Wait to arrive at score position
                if(!follower.isBusy()) {
                    needsAlignment = true;
                    alignmentTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2: // Align to goal
                if (alignToGoal() || alignmentTimer.getElapsedTimeSeconds() > 2.0) {
                    needsAlignment = false;
                    Shooting(true);
                    setPathState(3);
                }
                break;

            case 3: // Wait for preload shooting to finish
                if(shotsFired >= 3) {
                    Shooting(false);
                    follower.followPath(grabPickup1);
                    setPathState(4);
                }
                break;

            // ============================================
            // PICKUP 1 CYCLE (ONLY CYCLE IN THIS TEST)
            // ============================================
            case 4: // Wait to arrive at pickup location
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait to return to score position
                if(!follower.isBusy()) {
                    needsAlignment = true;
                    alignmentTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6: // Align to goal
                if (alignToGoal() || alignmentTimer.getElapsedTimeSeconds() > 2.0) {
                    needsAlignment = false;
                    Shooting(true);
                    setPathState(7);
                }
                break;

            case 7: // Wait for pickup1 shooting to finish
                if(shotsFired >= 3) {
                    Shooting(false);
                    setPathState(-1);  // DONE - Only 1 cycle
                }
                break;

            case -1: // Autonomous complete
                // All systems off, autonomous finished
                break;
        }
    }

    private void initializeHardware() {
        flicker = hardwareMap.get(Servo.class, "flicker");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotor.Direction.FORWARD);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType1 = flywheel1.getMotorType().clone();
        motorConfigurationType1.setAchieveableMaxRPMFraction(1.0);
        flywheel1.setMotorType(motorConfigurationType1);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Config.kP, Config.kI, Config.kD, Config.kF)
        );
        flywheel1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Config.kP, Config.kI, Config.kD, Config.kF)
        );

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);

        flicker.setPosition(Config.FLICKER_DEFAULT_POS);
        hood.setPosition(Config.HOOD_STARTING_POSITION);
    }

    public void Intake(boolean OnOff){
        intakeIsOn = OnOff;
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(intakeIsOn ? Config.INTAKE_POWER : 0.0);
    }

    public void Shooting(boolean OnOff){
        shootingIsOn = OnOff;
        if (shootingIsOn) {
            flywheel.setVelocity(Config.MEDIUMSHOTSPEED);
            flywheel1.setVelocity(Config.MEDIUMSHOTSPEED);
            shotsFired = 0;
            if (shootTimer != null) {
                shootTimer.resetTimer();
            }
        }
        else {
            flywheel.setVelocity(0);
            flywheel1.setVelocity(0);
        }
    }

    private void handleFlickerReload(boolean On) {
        if (On && !flickerReloading && shootTimer.getElapsedTimeSeconds() >= Config.SHOOTING_DELAY_SECONDS) {
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
        shotsFired++;
        if (shootTimer != null) {
            shootTimer.resetTimer();
        }
    }

    public void hood(){
        hood.setPosition(0.4);
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        if (!needsAlignment && localizationTimer.getElapsedTimeSeconds() > 0.5) {
            localizeViaAprilTag();
            localizationTimer.resetTimer();
        }

        if (shootingIsOn && shotsFired < 3) {
            handleFlickerReload(true);
        }

        // ============================================
        // TELEMETRY
        // ============================================
        telemetry.addData(">>> 1-CYCLE TEST MODE <<<", "");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Robot X", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Robot Y", String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("Robot Heading", String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Aligned to Goal", isAligned);
        telemetry.addData("Shots Fired", shotsFired + "/3");
        telemetry.addData("Shooting Active", shootingIsOn);
        telemetry.addData("Flicker Reloading", flickerReloading);
        telemetry.addData("Intake Active", intakeIsOn);
        telemetry.addData("AprilTags Detected", aprilTPR.getDetections().size());

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        localizationTimer = new Timer();
        alignmentTimer = new Timer();

        opmodeTimer.resetTimer();
        localizationTimer.resetTimer();

        initializeHardware();
        initializeAprilTag();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData(">>> 1-CYCLE TEST MODE <<<", "");
        telemetry.addData("Status", "Initialized - Ready to start!");
        telemetry.addData("Cycle", "Preload + Pickup 1 only");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        hood();
        Intake(true);
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}