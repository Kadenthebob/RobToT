package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class AutoGrabConstants {
    public static CustomPIDFCoefficients AutoMoveDrivePIDFCoefficients = new CustomPIDFCoefficients(
                0.0025,
                0.0016,
                0.0004,
                0);

    public static double autoGrabOffX = 0;
    public static double autoGrabOffY = 0;
    public static double autoGrabMaxPower = .3;
    public static boolean autoWait = false;
    public static double objLoopDistance = 15;

}
