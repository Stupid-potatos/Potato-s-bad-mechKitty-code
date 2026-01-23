package org.firstinspires.ftc.teamcode.Potato_Assets;

public class Config {
    public static final double FLICKER_DEFAULT_POS = 0.00;
    public static final double HOOD_STARTING_POSITION = 0.00;
    public static final double SHOOTING_DELAY_SECONDS = 0.3;
    // Servo positions
    public static final double FLICKER_DEFAULT = 0.13;
    public static final double FLICKER_MAX = 0.80;
    public static final double SERVO_INCREMENT = 0.02;
    public static final double HOOD_ADJUST_DELAY = 0.05;

    // Motor settings
    public static final double INTAKE_POWER = 0.3;
    public static final double SHOOTING_SPEED = 3000;

    // PIDF coefficients (~3-6% error)
    public static final double kP = 5.0;
    public static final double kI = 0.1;
    public static final double kD = 0.0;
    public static final double kF = 24;

    // Field positions (inches from center)
    public static final double BLUE_GOAL_X = -72;
    public static final double BLUE_GOAL_Y = -72;
    public static final double RED_GOAL_X = -72;
    public static final double RED_GOAL_Y = 72;
}
