package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = (48*Math.PI)/(2000*25.4);
        ThreeWheelConstants.strafeTicksToInches = (48*Math.PI)/(2000*25.4);
        ThreeWheelConstants.turnTicksToInches = (48*Math.PI)/(2000*25.4);
        ThreeWheelConstants.leftY = 6.03;
        ThreeWheelConstants.rightY = -6.03;
        ThreeWheelConstants.strafeX = -5.551;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFo2";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFo1";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightBo3";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




