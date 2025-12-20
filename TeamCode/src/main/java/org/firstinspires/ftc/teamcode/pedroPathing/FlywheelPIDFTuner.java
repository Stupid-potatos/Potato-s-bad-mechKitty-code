package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Potato_Assets.TuningController;

/**
 *  A TeleOp mode for tuning the PIDF on the flywheel motor using fullpanels.
 *  HEAVILY written (took and modified from Noah Bres)
 */
@Configurable
@TeleOp(name = "Flywheel PIDF Tuner", group = "Potato's testing")
public class FlywheelPIDFTuner extends LinearOpMode {

    // Configurable on fullpanels
    public static double kP = 5.0;
    public static double kI = 0.1;
    public static double kD = 0.0;
    public static double kF = 12.5;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {
        // Get motor from hardware map
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        // myMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable bulk caching for better performance
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Remove 85% speed limit
        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        // Enable encoder-based velocity control
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get battery voltage sensor for compensation
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Apply initial PIDF coefficients
        setPIDFCoefficients(myMotor, kP, kI, kD, kF);

        // Create tuning controller (automatically varies target velocity)
        TuningController tuningController = new TuningController();

        // Track last PIDF values to detect changes
        double lastKp = kP;
        double lastKi = kI;
        double lastKd = kD;
        double lastKf = kF;

        telemetry.addLine("=== Flywheel PIDF Tuner ===");
        telemetry.addLine("Ready to start!");
        telemetry.addLine();
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Open Panels");
        telemetry.addLine("2. Adjust kP, kI, kD, kF values");
        telemetry.addLine("3. Watch velocity tracking on graph");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Start automated velocity test
        tuningController.start();

        while (!isStopRequested() && opModeIsActive()) {
            // Get current target velocity from controller
            double targetVelocity = tuningController.update();

            // Command motor to target velocity
            myMotor.setVelocity(targetVelocity);

            // Get actual motor velocity
            double motorVelocity = myMotor.getVelocity();
            double error = targetVelocity - motorVelocity;
            double percentError = targetVelocity > 0 ? (error / targetVelocity) * 100 : 0;

            // Display telemetry
            telemetry.addLine("=== VELOCITY ===");
            telemetry.addData("Target", "%.0f ticks/sec", targetVelocity);
            telemetry.addData("Current", "%.0f ticks/sec", motorVelocity);
            telemetry.addData("Error", "%.0f ticks/sec (%.1f%%)", error, percentError);
            telemetry.addLine();

            telemetry.addLine("=== PIDF VALUES ===");
            telemetry.addData("P", "%.3f", kP);
            telemetry.addData("I", "%.3f", kI);
            telemetry.addData("D", "%.3f", kD);
            telemetry.addData("F", "%.3f", kF);
            telemetry.addLine();

            telemetry.addLine("=== PERFORMANCE ===");
            telemetry.addData("Status", getPerformanceIndicator(Math.abs(percentError)));
            telemetry.addData("Battery", "%.1fV", batteryVoltageSensor.getVoltage());

            // Dashboard graph bounds
            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(
                    TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            // Check if PIDF values changed and apply them
            if (lastKp != kP || lastKi != kI || lastKd != kD || lastKf != kF) {
                setPIDFCoefficients(myMotor, kP, kI, kD, kF);

                lastKp = kP;
                lastKi = kI;
                lastKd = kD;
                lastKf = kF;

                telemetry.addLine();
                telemetry.addLine("✓ PIDF values updated!");
            }

            telemetry.update();
        }

        // Stop motor when done
        myMotor.setPower(0);
    }

    private void setPIDFCoefficients(DcMotorEx motor, double p, double i, double d, double f) {
        // Apply voltage compensation to feedforward
        double compensatedF = f * 12.0 / batteryVoltageSensor.getVoltage();

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, i, d, compensatedF));
    }

    private String getPerformanceIndicator(double absPercentError) {
        if (absPercentError < 1) return "EXCELLENT";
        if (absPercentError < 3) return "GOOD";
        if (absPercentError < 5) return "OK";
        if (absPercentError < 10) return "NEEDS TUNING";
        return "X POOR - Keep Tuning";
    }
}