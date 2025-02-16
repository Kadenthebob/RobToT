package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = true;
        OTOSConstants.hardwareMapName = "otos";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(6, 0.1, 3*Math.PI / 2);
        OTOSConstants.linearScalar = 1;
        OTOSConstants.angularScalar = 1;

        PinpointConstants.forwardY = 6.125;
        PinpointConstants.strafeX = -5.551;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        PinpointConstants.customEncoderResolution = 2000/48;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}




