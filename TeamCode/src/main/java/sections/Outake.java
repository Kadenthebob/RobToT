package sections;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Outake extends Intake{
    @Config
    public static class ServoIntakeClawParams {
        public static double CLAW_OFFSET = 0;

        public static double CLAW_DEGREE_LIMIT = 85;//200
        public static double CLAW_DEGREE_MAX = 360*5;
        public static double CLAW_POWER_COEFF = 4;
    }

    @Config
    public static class ServoIntakeElbowParams {
        public static double ELBOW_OFFSET = 0;

        public static double ELBOW_DEGREE_LIMIT = 180;//200
        public static double ELBOW_DEGREE_MAX = 300;
        public static double ELBOW_POWER_COEFF = 4;
    }

    @Config
    public static class ServoIntakeDiffyParams {
        public static double DIFFY_DEGREE_MAX = 360*5;
        public static double TWIST_OFFSET = 90;

        public static double TWIST_DEGREE_LIMIT = 180;//200
        public static double TWIST_POWER_COEFF = 4;

        public static double WRIST_DEGREE_LIMIT = 20;//200

        public static double WRIST_POWER_COEFF = 4;
    }

    public Outake(HardwareMap hardwareMap){
        super(hardwareMap);
        claw = hardwareMap.get(ServoImplEx.class,"outClaw");
        claw.setPwmRange(Params.pwmRange);

        elbowR = hardwareMap.get(ServoImplEx.class,"outElbowR");
        elbowR.setDirection(Servo.Direction.REVERSE);
        elbowR.setPwmRange(Params.pwmRange);

        elbowL = hardwareMap.get(ServoImplEx.class,"outElbowL");
        elbowL.setDirection(Servo.Direction.FORWARD);
        elbowL.setPwmRange(Params.pwmRange);

        inDiffyR = hardwareMap.get(ServoImplEx.class,"outDiffyR");
        inDiffyR.setPwmRange(Params.pwmRange);
        inDiffyR.setDirection(Servo.Direction.REVERSE);

        inDiffyL = hardwareMap.get(ServoImplEx.class,"outDiffyL");
        inDiffyL.setPwmRange(Params.pwmRange);
        inDiffyL.setDirection(Servo.Direction.FORWARD);

        diffyParams.DIFFY_DEGREE_MAX = ServoIntakeDiffyParams.DIFFY_DEGREE_MAX;
        diffyParams.TWIST_DEGREE_LIMIT = ServoIntakeDiffyParams.TWIST_DEGREE_LIMIT;
        diffyParams.WRIST_DEGREE_LIMIT = ServoIntakeDiffyParams.WRIST_DEGREE_LIMIT;
    }
}
