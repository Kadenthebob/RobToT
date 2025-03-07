package sections;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Outake extends Intake{
    public Outake(HardwareMap hardwareMap){
        super(hardwareMap);
        claw = hardwareMap.get(ServoImplEx.class,"outClaw");
        claw.setPwmRange(Params.pwmRange);

        trunkR = hardwareMap.get(ServoImplEx.class,"outTrunkR");
        trunkR.setDirection(Servo.Direction.FORWARD);
        trunkR.setPwmRange(Params.pwmRange);

        trunkL = hardwareMap.get(ServoImplEx.class,"outTrunkL");
        trunkL.setDirection(Servo.Direction.REVERSE);
        trunkL.setPwmRange(Params.pwmRange);

        inDiffyR = hardwareMap.get(ServoImplEx.class,"outDiffyR");
        inDiffyR.setPwmRange(Params.pwmRange);
        inDiffyR.setDirection(Servo.Direction.FORWARD);

        inDiffyL = hardwareMap.get(ServoImplEx.class,"outDiffyL");
        inDiffyL.setPwmRange(Params.pwmRange);
        inDiffyL.setDirection(Servo.Direction.REVERSE);
    }
}
