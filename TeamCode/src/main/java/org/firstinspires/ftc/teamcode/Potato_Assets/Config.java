package org.firstinspires.ftc.teamcode.Potato_Assets;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Config {
    public static final Pose RED_NEAR_GOAL_START = new Pose(121,127,Math.toRadians(40));
    public static final Pose BLUE_NEAR_GOAL_START = new Pose(24,125,Math.toRadians(140));
    public static final Pose BLUE_FAR_START = new Pose(63,8,Math.toRadians(90));
    public static final Pose RED_FAR_START = new Pose(80,8,Math.toRadians(90));
    public static double FLICKER_DEFAULT_POS = 0.00;
    public static double HOOD_STARTING_POSITION = 0.00;
    public static double SHOOTING_DELAY_SECONDS = 0.3;
    // Servo positions
    public static double FLICKER_DEFAULT = 0.13;
    public static double FLICKER_MAX = 0.80;
    public static double SERVO_INCREMENT = 0.02;
    public static double HOOD_ADJUST_DELAY = 0.05;

    // Motor settings
    public static double INTAKE_POWER = 0.3;
    public static double FARSHOTSPEED = 3000;
    public static double CLOSESHOTSPEED = 4000;
    public static double MEDIUMSHOTSPEED = 5000;
    public static double MAX_HOOD_ANGLE = 45;
    public static double SHOOTINGSPEEDCURRENT = 3000;

    // PIDF coefficients (~3-6% error)
    public static final double kP = 5.0;
    public static final double kI = 0.1;
    public static final double kD = 0.0;
    public static final double kF = 24;

    // Field positions (inches from center)
    public static double BLUE_GOAL_X = 11;
    public static double BLUE_GOAL_Y = 135;
    public static double RED_GOAL_X = 133;
    public static double RED_GOAL_Y = 135;
}
