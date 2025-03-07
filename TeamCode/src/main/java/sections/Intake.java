package sections;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        public double trunkMaxDegree = 190;//200
        public double trunkServoMaxTurn = (360*5-180)/2;
        public double trunkPowerCoeff = 2;

        public double armMaxDegree = 300;//200
        public double armServoMaxTurn = 300;
        public double armPowerCoeff = 5;

        public double clawMaxDegree = 85;//200
        public double clawServoMaxTurn = 360*5-180;
        public double clawPowerCoeff = 4;

    }
    @Config
    public static class ServoTwistParams {
        public static double TWIST_OFFSET = 90;

        public static double TWIST_DEGREE_LIMIT = 180;//200
        public static double TWIST_DEGREE_MAX = 300/2;
        public static double TWIST_POWER_COEFF = 4;
    }

    @Config
    public static class ServoIntakeDiffyParams {
        public static double DIFFY_DEGREE_MAX = 150;
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

    ServoTwistParams twistParams = new ServoTwistParams();

    ServoIntakeDiffyParams diffyParams = new ServoIntakeDiffyParams();

    Params PARAMS = new Params();
    CRServo intakeR, intakeL;
    public ServoImplEx trunkR, trunkL, claw, inDiffyR, inDiffyL;
    public Intake(HardwareMap hardwareMap){
        claw = hardwareMap.get(ServoImplEx.class,"inClaw");
        claw.setPwmRange(Params.pwmRange);

        trunkR = hardwareMap.get(ServoImplEx.class,"inTrunkR");
        trunkR.setDirection(Servo.Direction.FORWARD);
        trunkR.setPwmRange(Params.pwmRange);

        trunkL = hardwareMap.get(ServoImplEx.class,"inTrunkL");
        trunkL.setDirection(Servo.Direction.REVERSE);
        trunkL.setPwmRange(Params.pwmRange);

        inDiffyR = hardwareMap.get(ServoImplEx.class,"inDiffyR");
        inDiffyR.setPwmRange(Params.pwmRange);
        inDiffyR.setDirection(Servo.Direction.FORWARD);

        inDiffyL = hardwareMap.get(ServoImplEx.class,"inDiffyL");
        inDiffyL.setPwmRange(Params.pwmRange);
        inDiffyL.setDirection(Servo.Direction.REVERSE);

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
        double pos = Math.max(Math.min(degree,PARAMS.clawMaxDegree),0); //sets a limit to what you can set the servo position to go to

        pos/=PARAMS.clawServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)

        claw.setPosition(pos);
    }
    public Action SetClawClose(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setClawPos(0);
                clawOpen = false;
                return false;
            }
        };
    }

    public Action SetClawOpen(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setClawPos(65);
                clawOpen = true;
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

    public void setTrunkWall(){
        double degree = 10;
        double pos = Math.max(Math.min(degree,PARAMS.trunkMaxDegree),0); //sets a limit to what you can set the servo position to go to

        pos/=PARAMS.trunkServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)

        trunkR.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
        trunkL.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
    }

    public void setTrunkHoop(){
        double degree = 60;
        double pos = Math.max(Math.min(degree,PARAMS.trunkMaxDegree),0); //sets a limit to what you can set the servo position to go to

        pos/=PARAMS.trunkServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)

        trunkR.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
        trunkL.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
    }

    public void setTrunkPower(double pow){
        if(!autoOveride) {
            armPos = PARAMS.trunkServoMaxTurn * (trunkR.getPosition()) - trunkOffset;
            setTrunkPos(armPos + pow * 4);
        }
    }

    public void setTrunkPos(double degree){
        double pos = Math.max(Math.min(degree,PARAMS.trunkMaxDegree),0); //sets a limit to what you can set the servo position to go to

        pos/=PARAMS.trunkServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)

        trunkR.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
        trunkL.setPosition(pos+(trunkOffset/PARAMS.trunkServoMaxTurn));
    }
    public Action SetTrunkPos(double degree){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setTrunkPos(degree);
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
        trunkL.setPwmDisable();
        trunkR.setPwmDisable();
        inDiffyL.setPwmDisable();
        inDiffyR.setPwmDisable();
    }




}
