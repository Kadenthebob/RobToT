package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class AutoGrabConstants {
    public static CustomPIDFCoefficients AutoMoveDrivePIDFCoefficients = new CustomPIDFCoefficients(
                0.002,
                0.000,
                .00006,
                0);

    public static double autoGrabOffX = -4;
    public static double autoGrabOffY = -3.1;
    public static double autoGrabMaxPower = .5;
    public static boolean autoWait = false;
    public static double objLoopDistance = 150;

}
