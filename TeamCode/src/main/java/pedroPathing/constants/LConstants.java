package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = (48*Math.PI)/(2000*25.4);
        ThreeWheelIMUConstants.strafeTicksToInches = (48*Math.PI)/(2000*25.4);
        ThreeWheelIMUConstants.turnTicksToInches = (48*Math.PI)/(2000*25.4);
        ThreeWheelIMUConstants.leftY = 6.03;
        ThreeWheelIMUConstants.rightY = -6.03;
        ThreeWheelIMUConstants.strafeX = -5.551;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftFo2";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightFo1";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "rightBo3";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




