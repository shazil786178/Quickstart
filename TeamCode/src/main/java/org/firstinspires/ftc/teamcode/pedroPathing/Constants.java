package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//translational, drive, heading, centripetal PIDF need to be done.
//https://pedropathing.com/docs/pathing/tuning/pids/translational
// Link teaches different ways of tuning but not sure if we do that first then the PIDF stuff:
//https://www.ctrlaltftc.com/the-pid-controller/tuning-methods-of-a-pid-controller
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .xVelocity(59.85300637047654)
            .yVelocity(48.269750941760826)
            //test commit
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-33.082915508877946)
            .lateralZeroPowerAcceleration(-55.46050353712529)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.005, 0.025));
            //update the heading bc it dosent work

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1)
            .rightPodY(-1)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("backLeft")
            .rightEncoder_HardwareMapName("backRight")//change
            .strafeEncoder_HardwareMapName("frontRight")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                /* other builder steps */
                .build();
    }
}
