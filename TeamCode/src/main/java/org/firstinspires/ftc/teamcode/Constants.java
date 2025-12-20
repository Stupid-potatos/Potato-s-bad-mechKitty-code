package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15);
            //.forwardZeroPowerAcceleration() put in value from forward zero power acceleration tuner
            //.lateralZeroPowerAcceleration() put in value from lateral zero power acceleration tuner
            //.translationalPIDFCoefficients(new PIDFCoefficients(P , I , D , F))    put in value from translational tuner
            //.headingPIDFCoefficients(new PIDFCoefficients(P , I , D , F))          put in value from heading tuner
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(P , I , D , F))    put in value from drive tuner
            //.centripetalScaling()           put in value from centripetal tuner

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
            // .xVelocity()      PUT IN VELOCITY GIVEN FROM TUNING CLASS (Forward velocity tuner)
            // .yVelocity()      PUT IN VELOCITY GIVEN FROM TUNING CLASS (Lateral velocity tuner)

    public static PinpointConstants  localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.CM)
            // Physical pod positions (measure from center of rotation)
            //.forwardPodY()   // How far forward/back the forward pod is (in inches)
            //.strafePodX()    // How far left/right the strafe pod is (in inches)

            // Which goBILDA pods you're using
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // Encoder directions (adjust if tracking backwards)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

            // Uncomment after tuning:
            //.forwardTicksToInches(multiplier)  // VALUE FROM FORWARD TUNER
            //.strafeTicksToInches(multiplier)   // VALUE FROM STRAFE TUNER


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    );
            //
            //
            //  lower the value the less you overshoot, the higher the value the more you overshoot but more abruptly
            //

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

