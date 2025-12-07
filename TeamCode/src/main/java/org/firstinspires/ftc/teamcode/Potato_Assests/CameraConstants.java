package org.firstinspires.ftc.teamcode.Potato_Assests;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Camera configuration constants for AprilTag detection
 * Contains values for camera setup and calibration
 */
public class CameraConstants {
    // Camera physical position relative to robot center (inches)
    // X = right/left, Y = forward/back, Z = up/down
    public static final Position CAMERA_POSITION = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);

    // Camera orientation angles relative to robot (degrees)
    // Yaw = rotation, Pitch = up/down tilt, Roll = side tilt
    public static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0);

    // Camera calibration/intrinsic parameters (in pixels)
    public static final double CAMERA_FX = 115.612000761;    // Focal length X
    public static final double CAMERA_FY = 115.612000761;    // Focal length Y
    public static final double CAMERA_CX = 395.22208662;     // Optical center X
    public static final double CAMERA_CY = 300.762956164;    // Optical center Y

    // Camera resolution
    public static final int CAMERA_WIDTH = 640;    // Image width (pixels)
    public static final int CAMERA_HEIGHT = 480;   // Image height (pixels)

    // Hardware configuration name (must match robot config)
    public static final String CAMERA_NAME = "Webcam 1";

    // AprilTag detection settings
    public static final double DECIMATION = 0.5F;  // Lower = longer range but slower

    // Tag filtering - ignore tags containing this string
    public static final String TAG_FILTER = "Obelisk";
}