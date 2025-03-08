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

public class Intake {
    public static class Params {

        public static PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500,2500);

        double intakeSpeed = 1;

    }
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
        public static double DIFFY_DEGREE_MAX = 300;
        public static double TWIST_OFFSET = 90;

        public static double TWIST_DEGREE_LIMIT = 180;//200
        public static double TWIST_POWER_COEFF = 4;

        public static double WRIST_DEGREE_LIMIT = 20;//200

        public static double WRIST_POWER_COEFF = 4;
    }

    double trunkPos = 0;

    double avgDiffyPos = 0;
    double wristPos = 0;
    double twistPos = 0;
    double armPos = 0;
    double elbowOffest = 0;
    double trunkOffset = 0;

    boolean autoOveride = false;
    boolean clawOpen = false;


    public ServoIntakeClawParams clawParams = new ServoIntakeClawParams();
    public ServoIntakeElbowParams elbowParams = new ServoIntakeElbowParams();
    public ServoIntakeDiffyParams diffyParams = new ServoIntakeDiffyParams();

    Params PARAMS = new Params();
    CRServo intakeR, intakeL;
    public ServoImplEx elbowR, elbowL, claw, inDiffyR, inDiffyL;
    public Intake(HardwareMap hardwareMap){
        claw = hardwareMap.get(ServoImplEx.class,"inClaw");
        claw.setPwmRange(Params.pwmRange);

        elbowR = hardwareMap.get(ServoImplEx.class,"inElbowR");
        elbowR.setDirection(Servo.Direction.REVERSE);
        elbowR.setPwmRange(Params.pwmRange);

        elbowL = hardwareMap.get(ServoImplEx.class,"inElbowL");
        elbowL.setDirection(Servo.Direction.FORWARD);
        elbowL.setPwmRange(Params.pwmRange);

        inDiffyR = hardwareMap.get(ServoImplEx.class,"inDiffyR");
        inDiffyR.setPwmRange(Params.pwmRange);
        inDiffyR.setDirection(Servo.Direction.REVERSE);

        inDiffyL = hardwareMap.get(ServoImplEx.class,"inDiffyL");
        inDiffyL.setPwmRange(Params.pwmRange);
        inDiffyL.setDirection(Servo.Direction.FORWARD);

    }

    public void updateDiffyPos(){
        double diffyRPos = inDiffyR.getPosition()*diffyParams.DIFFY_DEGREE_MAX;
        double diffyLPos = inDiffyL.getPosition()*diffyParams.DIFFY_DEGREE_MAX;

        avgDiffyPos = (diffyLPos + diffyRPos)/2;
        twistPos = (diffyRPos-avgDiffyPos)*2;
        wristPos = diffyParams.TWIST_OFFSET - avgDiffyPos;
    }
    public void setDiffyWristPos(double pos){
        updateDiffyPos();
        pos = MathFunctions.clamp(pos,0,diffyParams.WRIST_DEGREE_LIMIT);
        setDiffyPos(pos, twistPos);
    }

    public void setDiffyTwistPos(double pos){
        updateDiffyPos();
        pos = MathFunctions.clamp(pos,0,diffyParams.TWIST_DEGREE_LIMIT);
        setDiffyPos(wristPos, pos);
    }

    private void setDiffyPos(double wrist, double twist){
        wrist/=diffyParams.DIFFY_DEGREE_MAX;
        twist/=diffyParams.DIFFY_DEGREE_MAX;
        twist/=2;
        double off = diffyParams.TWIST_OFFSET/diffyParams.DIFFY_DEGREE_MAX;
        inDiffyR.setPosition(wrist+twist+off);
        inDiffyL.setPosition(wrist-twist+off);
    }

    public void diffyReset(){
        inDiffyR.setPosition(90/diffyParams.DIFFY_DEGREE_MAX);
        inDiffyL.setPosition(90/diffyParams.DIFFY_DEGREE_MAX);
    }

    public void setClawPos(double degree){
        double pos = Math.max(Math.min(degree,clawParams.CLAW_DEGREE_LIMIT),0); //sets a limit to what you can set the servo position to go to

        pos/=clawParams.CLAW_DEGREE_MAX; //converts from degrees(0-360) to servo position(0-1)

        claw.setPosition(pos);
    }

    public void setClawOpen(){
        setClawPos(65);
        clawOpen = true;
    }

    public void setClawClose(){
        setClawPos(65);
        clawOpen = true;
    }
    public Action SetClawClose(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setClawClose();
                return false;
            }
        };
    }

    public Action SetClawOpen(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setClawOpen();
                return false;
            }
        };
    }

    public void setClawAutoOpen(Camera cam){
        if(cam.getSelectedSurrounded()){
            setClawPos(0);
            clawOpen = false;
        }else{
            setClawPos(65);
            clawOpen = true;
        }
    }

    public void setClawAutoClose(Camera cam){
        if(cam.getSelectedSurrounded()){
            setClawPos(65);
            clawOpen = true;
        }else{
            setClawPos(0);
            clawOpen = false;
        }
    }

    public Action SetClawAutoOpen(Camera cam){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setClawAutoOpen(cam);
                return false;
            }
        };
    }

    public Action SetClawAutoClose(Camera cam){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setClawAutoClose(cam);
                return false;
            }
        };
    }

    public void toggleClaw(){
        if(clawOpen){
            setClawPos(0);
            clawOpen = false;
        } else{
            setClawPos(65);
            clawOpen = true;
        }
    }
//    public void setTrunkPit(){
//        double degree = 190;
//        double pos = Math.max(Math.min(degree,PARAMS.trunkMaxDegree),0); //sets a limit to what you can set the servo position to go to
//
//        pos/=PARAMS.trunkServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)
//        elbowOffest = 0;
//        setElbowPos(100);
//        setTwistPos(90);
//        trunkR.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
//        trunkL.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
//    }
//    public void setTrunkZero(){
//        double degree = 0;
//        double pos = Math.max(Math.min(degree,PARAMS.trunkMaxDegree),0); //sets a limit to what you can set the servo position to go to
//
//        pos/=PARAMS.trunkServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)
//        elbowOffest = 0;
//        setElbowPos(100);
//        setTwistPos(90);
//        trunkR.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
//        trunkL.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
//    }

    public void setElbowPos(double degree){
        double pos = Math.max(Math.min(degree,elbowParams.ELBOW_DEGREE_LIMIT),0); //sets a limit to what you can set the servo position to go to

        pos/=elbowParams.ELBOW_DEGREE_MAX; //converts from degrees(0-360) to servo position(0-1)

        elbowR.setPosition(pos+(elbowParams.ELBOW_OFFSET/elbowParams.ELBOW_DEGREE_MAX));
        elbowL.setPosition(pos+(elbowParams.ELBOW_OFFSET/elbowParams.ELBOW_DEGREE_MAX));
    }
    public Action SetElbowPos(double degree){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setElbowPos(degree);
                return false;
            }
        };
    }


//    public Action SetTrunkPit(){
//        return new SequentialAction(
//                SetTwistPos(0),
//                SetElbowPos(100),
//                SetTrunkPos(175),
//                new SleepAction(1),
//                SetElbowPos(23)
//        );
//    }
//
//    public Action SetTrunkWall(){
//        return new SequentialAction(
//                SetTwistPos(0),
//                SetElbowPos(100),
//                SetTrunkPos(20),
//                new SleepAction(.5),
//                SetElbowPos(300),
//                new SleepAction(.8),
//                SetTwistPos(90)
//        );
//    }
//
//    public Action SetTrunkHoop(){
//        return new SequentialAction(
//                SetTwistPos(90),
//                SetElbowPos(100),
//                SetTrunkPos(60),
//                new SleepAction(.5),
//                SetElbowPos(300),
//                new SleepAction(.8),
//                SetTwistPos(90)
//        );
//    }
//
//    public Action SetSpec(Lifters lift){
//        return new SequentialAction(
//                SetTwistPos(90),
//                SetElbowPos(115),
//                SetTrunkPos(100),
//                lift.setVertLifterPos(1200,1)
//        );
//    }
//
//    public Action SetSpecRelease(Lifters lift){
//        return new SequentialAction(
//                lift.setVertLifterPos(1900,1),
//                SetClawOpen()
//        );
//    }
//
//    public Action SpecHold(Lifters lift){
//        return new ParallelAction(
//                lift.setVertLifterPos(1180,1),
//                SetTwistPos(90),
//                SetElbowPos(157.8),
//                SetTrunkPos(153.4)
//
//        );
//    }
//
//    public Action SpecRelease(Lifters lift){
//        return new SequentialAction(
//                SetTwistPos(90),
//                SetElbowPos(184.2),
//                SetTrunkPos(97.2)
//
//        );
//    }
//
//    public void setTwistPower(double pow){
//        if(!autoOveride) {
//            twistPos = twist.getPosition() * twistParams.TWIST_DEGREE_MAX - twistParams.TWIST_OFFSET;
//            setTwistPos(twistPos + pow * twistParams.TWIST_POWER_COEFF);
//        }
//    }

    public void setTwistMatchObjAngle(Camera cam,boolean ninetyOffset){
        updateDiffyPos();
        double pos = -twistPos;
        double off = 0;
        if(!ninetyOffset) off += 90;
        double serv = (180-(cam.getObjRot()+90)+off-pos)%180;
//        double serv = (180-(cam.getObjRot()+90-pos)+off)%180;
        if(serv<0){
            serv+=180;
        }
        setDiffyTwistPos(serv);
    }

    public void setTwistMatchObjAngle(Camera cam){
        setTwistMatchObjAngle(cam, !cam.getSelectedSurrounded());
    }

    public Action SetTwistPos(double degree){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setDiffyTwistPos(degree);
                return false;
            }
        };
    }

    public Action SetTwistMatchObjAngle(Camera cam,boolean ninetyOffset){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setTwistMatchObjAngle(cam,ninetyOffset);
                return false;
            }
        };
    }
    public Action SetTwistMatchObjAngle(Camera cam){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setTwistMatchObjAngle(cam);
                return false;
            }
        };
    }

    public Action autoOverideOff(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                autoOveride = false;
                return false;
            }
        };
    }

    public Action autoOverideOn(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                autoOveride = false;
                return false;
            }
        };
    }

    public void turnOffServos(){
        claw.setPwmDisable();
        elbowL.setPwmDisable();
        elbowR.setPwmDisable();
        inDiffyL.setPwmDisable();
        inDiffyR.setPwmDisable();
    }




}
