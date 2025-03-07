package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFo2";
        FollowerConstants.leftRearMotorName = "leftB";
        FollowerConstants.rightFrontMotorName = "rightFo1";
        FollowerConstants.rightRearMotorName = "rightBo3";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.56;

        FollowerConstants.xMovement = 75.1;
        FollowerConstants.yMovement = 58.2;

        FollowerConstants.forwardZeroPowerAcceleration = -48.008;
        FollowerConstants.lateralZeroPowerAcceleration = -77.9724;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.18,0,0.019,0);
        FollowerConstants.translationalPIDFFeedForward = 0.024;

        FollowerConstants.translationalPIDFSwitch = 3;
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.275,0,0.035,0);
        FollowerConstants.secondaryDrivePIDFFeedForward = 0.024;// Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.5,0,0.11,0);
        FollowerConstants.headingPIDFFeedForward = 0;

        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0063,0,0.002,0.6,0);
        FollowerConstants.drivePIDFFeedForward = 0;

        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.5;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.25;
        FollowerConstants.pathEndTranslationalConstraint = 0.25;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.nominalVoltage = 12;
    }
}
