package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            .translationalPIDFCoefficients(new PIDFCoefficients(0.01,0, 0.045, 0.045))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.01,0,0.045,0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5,0,0,0,0.5))
            .forwardZeroPowerAcceleration(-35.851226940289387)
            .lateralZeroPowerAcceleration(-69.36796589713219)
//            .useSecondaryTranslationalPIDF(true)
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
            .mass(9.9);






    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(59.751979821575408)
            .yVelocity(40.771475445622558)
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.003005409314507831)
            .strafeTicksToInches(0.002912071518331959)
            .turnTicksToInches(0.0029028733755955983)
            .leftPodY(5.125)
            .rightPodY(-5.125)
            .strafePodX(7.375)

            .leftEncoder_HardwareMapName("FL")
            .rightEncoder_HardwareMapName("BR")
            .strafeEncoder_HardwareMapName("FR")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
}
